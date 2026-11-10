"""compare_oracle.py [--no-build] [--map-check PATH] [map ...]

Runs the C++ map parser (map_check) and the ValveResourceFormat reference
(tools/MapOracle) over every installed map - official maps and workshop items
in every Steam library - and diffs their summaries ("  info" lines ignored).
Run it after a CS2 update or a VRF update: any difference means the parser is
wrong or out of date.  Exit code 0 = every map matches, 77 = no CS2 install.

Finds the game like the C++ tests: S2GHTLINE_CS2_DIR (the game's folder or
its game/csgo folder), else every library in Steam's libraryfolders.vdf.
map_check: --map-check, else S2GHTLINE_MAP_CHECK, else the newest one under
build/.  Needs the .NET 10 SDK for the oracle.
"""
import glob, os, re, struct, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ORACLE = os.path.join(ROOT, 'tools', 'MapOracle')
GAME = os.path.join('steamapps', 'common', 'Counter-Strike Global Offensive')
SKIPPED = 77


def steam_roots():
    roots = []
    if sys.platform == 'win32':
        for var in ('ProgramFiles(x86)', 'ProgramFiles'):
            if os.environ.get(var):
                roots.append(os.path.join(os.environ[var], 'Steam'))
        try:
            import winreg
            with winreg.OpenKey(winreg.HKEY_CURRENT_USER, r'Software\Valve\Steam') as key:
                roots.append(os.path.normpath(winreg.QueryValueEx(key, 'SteamPath')[0]))
        except OSError:
            pass
    else:
        home = os.path.expanduser('~')
        roots += [os.path.join(home, '.steam', 'steam'), os.path.join(home, '.local', 'share', 'Steam'),
                  os.path.join(home, '.var', 'app', 'com.valvesoftware.Steam', '.local', 'share', 'Steam')]
    return [r for r in roots if os.path.isdir(r)]


def steam_libraries():
    """Every Steam library (folders holding steamapps/), each once."""
    libs = []
    def add(path):
        if os.path.isdir(path) and os.path.normcase(os.path.realpath(path)) not in map(
                lambda p: os.path.normcase(os.path.realpath(p)), libs):
            libs.append(path)
    for root in steam_roots():
        add(root)
        vdf = os.path.join(root, 'steamapps', 'libraryfolders.vdf')
        if os.path.exists(vdf):
            for path in re.findall(r'"path"\s+"([^"]+)"', open(vdf, encoding='utf-8', errors='replace').read()):
                add(path.replace('\\\\', '\\'))
    return libs


def game_dirs():
    """The game's game/csgo folders: S2GHTLINE_CS2_DIR's, else every library's."""
    override = os.environ.get('S2GHTLINE_CS2_DIR')
    if override:
        for d in (override, os.path.join(override, 'game', 'csgo')):
            if os.path.isfile(os.path.join(d, 'pak01_dir.vpk')):
                return [d]
        sys.exit(f'S2GHTLINE_CS2_DIR={override} is not a CS2 install (no game/csgo/pak01_dir.vpk)')
    dirs = [os.path.join(lib, GAME, 'game', 'csgo') for lib in steam_libraries()]
    return [d for d in dirs if os.path.isfile(os.path.join(d, 'pak01_dir.vpk'))]


def vpk_paths(path):
    """The file paths in a VPK's directory tree (enough to find nested map VPKs)."""
    with open(path, 'rb') as f:
        head = f.read(12)
        sig, ver, tree = struct.unpack('<III', head)
        if sig != 0x55AA1234:
            return []
        start = 28 if ver == 2 else 12
        f.seek(0)
        b = f.read(start + tree)
    def cstr(o):
        e = b.index(b'\0', o)
        return b[o:e].decode('utf-8', 'replace'), e + 1
    o, out = start, []
    while True:
        ext, o = cstr(o)
        if not ext:
            return out
        while True:
            d, o = cstr(o)
            if not d:
                break
            while True:
                n, o = cstr(o)
                if not n:
                    break
                pre = struct.unpack_from('<H', b, o + 4)[0]
                o += 18 + pre
                out.append((n + '.' + ext) if d == ' ' else f'{d}/{n}.{ext}')


def find_maps(csgo_dirs):
    """(vpk path, map name) for every official map with physics and every workshop map."""
    maps = []
    for csgo in csgo_dirs:
        official = os.path.join(csgo, 'maps')
        if os.path.isdir(official):
            for f in sorted(os.listdir(official)):
                name = f[:-4]
                if f.endswith('.vpk') and 'vanity' not in name and name != 'graphics_settings':
                    maps.append((os.path.join(official, f), name))
        # <library>/steamapps/common/<game>/game/csgo -> <library>/steamapps/workshop/content/730
        steamapps = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(csgo))))
        workshop = os.path.join(steamapps, 'workshop', 'content', '730')
        if os.path.isdir(workshop):
            for item in sorted(os.listdir(workshop)):
                folder = os.path.join(workshop, item)
                if not os.path.isdir(folder):
                    continue
                for f in os.listdir(folder):
                    if not f.endswith('.vpk') or re.search(r'_\d{3}\.vpk$', f):
                        continue   # archives are read through their _dir file
                    vpk = os.path.join(folder, f)
                    for p in vpk_paths(vpk):
                        m = re.fullmatch(r'maps/([^/]+)\.vpk', p)
                        if m and not m.group(1).endswith('_3dsky'):
                            maps.append((vpk, m.group(1)))
    # A game installed in two libraries lists every map twice: keep the first.
    seen, unique = set(), []
    for vpk, name in maps:
        if name not in seen:
            seen.add(name)
            unique.append((vpk, name))
    return unique


def find_map_check():
    if os.environ.get('S2GHTLINE_MAP_CHECK'):
        return os.environ['S2GHTLINE_MAP_CHECK']
    exe = 'map_check.exe' if sys.platform == 'win32' else 'map_check'
    found = glob.glob(os.path.join(ROOT, 'build', '**', exe), recursive=True)
    if not found:
        sys.exit('map_check not found: build it (cmake --build --preset release) or pass --map-check PATH')
    return max(found, key=os.path.getmtime)


def summaries(cmd):
    """map -> list of compared lines."""
    out = subprocess.run(cmd, capture_output=True, text=True).stdout
    result, cur = {}, None
    for line in out.splitlines():
        if line.startswith('map '):
            cur = line[4:].strip()
            result[cur] = []
        elif cur and not line.startswith('  info'):
            result[cur].append('  FAILED' if line.startswith('  FAILED') else line)
    return result


def main():
    csgo_dirs = game_dirs()
    if not csgo_dirs:
        print('SKIPPED: no CS2 install found: set S2GHTLINE_CS2_DIR to the game\'s folder')
        return SKIPPED

    args = sys.argv[1:]
    if '--map-check' in args:
        i = args.index('--map-check')
        map_check, args = args[i + 1], args[:i] + args[i + 2:]
    else:
        map_check = find_map_check()
    map_check = os.path.abspath(map_check)
    names = [a for a in args if not a.startswith('--')]
    if '--no-build' not in args:
        subprocess.run(['dotnet', 'build', ORACLE, '-c', 'Release', '-v', 'quiet', '-nologo'], check=True)
    maps = [m for m in find_maps(csgo_dirs) if not names or m[1] in names]
    print(f'map_check: {map_check}')
    print(f'{len(maps)} maps in {", ".join(csgo_dirs)}')
    flat = [x for vpk, name in maps for x in (vpk, name)]
    cpp = summaries([map_check] + flat)
    ref = summaries(['dotnet', 'run', '--project', ORACLE, '-c', 'Release', '--no-build', '--'] + flat)

    bad = 0
    for _, name in maps:
        a, b = cpp.get(name), ref.get(name)
        if a == b:
            print(f'  ok    {name}' + ('  (both failed)' if a == ['  FAILED'] else ''))
            continue
        bad += 1
        print(f'  DIFF  {name}')
        for x, y in zip((a or []) + [''] * 40, (b or []) + [''] * 40):
            if x != y:
                print(f'          c++: {x.strip()}\n          vrf: {y.strip()}')
                break
    print(f'{len(maps) - bad} / {len(maps)} maps match')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
