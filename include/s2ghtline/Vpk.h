#pragma once

#include "Bytes.h"

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

// Vpk.h — reads files out of a Valve pak (VPK v1 / v2).
//
// Layout: header, then the directory tree (extension → directory → name,
// each entry {crc, preload bytes, archive, offset, length} followed by its
// preload bytes), then the file data.  Archive 0x7FFF = the data sits in this
// file after the tree; any other index = `<name>_NNN.vpk` beside `<name>_dir.vpk`
// (workshop items).  A map's VPK is a single file; a workshop item nests the
// map's VPK inside its own, so a VPK can also be opened from memory.
//
// Only the header and tree are read on open; file data is read per request.
namespace s2ghtline {

    class Vpk {
    public:
        // A VPK on disk (`x.vpk`, or `x_dir.vpk` with `x_NNN.vpk` archives).
        [[nodiscard]] static std::optional<Vpk> open(const std::filesystem::path& file, std::string& why) {
            std::vector<uint8_t> header;
            if (!readRange(file, 0, kHeaderV2, header) && !readRange(file, 0, kHeaderV1, header)) {
                why = "can't read " + file.string();
                return std::nullopt;
            }
            const auto layout = parseHeader(ByteView(header), why);
            if (!layout)
                return std::nullopt;

            Vpk vpk;
            vpk.m_file = file;
            if (!readRange(file, 0, layout->dataStart, vpk.m_bytes)) {
                why = "VPK directory truncated";
                return std::nullopt;
            }
            if (!vpk.index(*layout, why))
                return std::nullopt;
            return vpk;
        }

        // A VPK held in memory: single-file only (its data follows the tree).
        [[nodiscard]] static std::optional<Vpk> fromMemory(std::vector<uint8_t> bytes, std::string& why) {
            const auto layout = parseHeader(ByteView(bytes), why);
            if (!layout)
                return std::nullopt;
            Vpk vpk;
            vpk.m_bytes    = std::move(bytes);
            vpk.m_inMemory = true;
            if (!vpk.index(*layout, why))
                return std::nullopt;
            return vpk;
        }

        [[nodiscard]] bool contains(std::string_view path) const { return m_entries.contains(std::string(path)); }

        // Every file's path ("maps/de_mirage/world_physics.vmdl_c").
        template <class F>
        void forEachPath(F&& f) const {
            for (const auto& [path, entry] : m_entries)
                f(std::string_view(path));
        }

        [[nodiscard]] std::optional<std::vector<uint8_t>> read(std::string_view path, std::string& why) const {
            const auto it = m_entries.find(std::string(path));
            if (it == m_entries.end()) {
                why = "not in the VPK: " + std::string(path);
                return std::nullopt;
            }
            const Entry& e = it->second;

            std::vector<uint8_t> out(m_bytes.begin() + static_cast<ptrdiff_t>(e.preloadAt),
                                     m_bytes.begin() + static_cast<ptrdiff_t>(e.preloadAt + e.preloadSize));
            if (e.length == 0)
                return out;

            std::vector<uint8_t> body;
            if (!readBody(e, body, why))
                return std::nullopt;
            out.insert(out.end(), body.begin(), body.end());
            return out;
        }

    private:
        static constexpr uint32_t kSignature    = 0x55AA1234;
        static constexpr size_t   kHeaderV1     = 12;
        static constexpr size_t   kHeaderV2     = 28;
        static constexpr uint16_t kThisArchive  = 0x7FFF;
        static constexpr uint16_t kEntryEnd     = 0xFFFF;
        static constexpr uint32_t kMaxTreeSize  = 512u << 20;
        static constexpr uint32_t kMaxEntrySize = 1u << 30;

        struct Layout {
            size_t treeStart = 0;
            size_t dataStart = 0;   // = end of the tree
        };

        struct Entry {
            size_t   preloadAt   = 0;   // in m_bytes
            uint16_t preloadSize = 0;
            uint16_t archive     = 0;
            uint32_t offset      = 0;
            uint32_t length      = 0;
        };

        std::filesystem::path                  m_file;        // empty when in memory
        std::vector<uint8_t>                   m_bytes;       // header + tree (on disk) / the whole file (in memory)
        size_t                                 m_dataStart = 0;
        bool                                   m_inMemory  = false;
        std::unordered_map<std::string, Entry> m_entries;

        [[nodiscard]] static std::optional<Layout> parseHeader(ByteView bytes, std::string& why) {
            ByteCursor c(bytes);
            const auto signature = c.read<uint32_t>();
            const auto version   = c.read<uint32_t>();
            const auto treeSize  = c.read<uint32_t>();
            if (!c.ok() || signature != kSignature) {
                why = "not a VPK";
                return std::nullopt;
            }
            if (version != 1 && version != 2) {
                why = "unsupported VPK version " + std::to_string(version);
                return std::nullopt;
            }
            if (treeSize > kMaxTreeSize) {
                why = "VPK directory too large";
                return std::nullopt;
            }
            const size_t treeStart = version == 2 ? kHeaderV2 : kHeaderV1;
            return Layout{ treeStart, treeStart + treeSize };
        }

        bool index(const Layout& layout, std::string& why) {
            m_dataStart = layout.dataStart;
            const auto tree = ByteView(m_bytes).sub(0, layout.dataStart);
            if (!tree) {
                why = "VPK directory truncated";
                return false;
            }
            ByteCursor c(*tree);
            c.skip(layout.treeStart);

            for (std::string_view ext = c.cstr(); c.ok() && !ext.empty(); ext = c.cstr()) {
                for (std::string_view dir = c.cstr(); c.ok() && !dir.empty(); dir = c.cstr()) {
                    for (std::string_view name = c.cstr(); c.ok() && !name.empty(); name = c.cstr()) {
                        Entry e;
                        c.read<uint32_t>();   // crc
                        e.preloadSize = c.read<uint16_t>();
                        e.archive     = c.read<uint16_t>();
                        e.offset      = c.read<uint32_t>();
                        e.length      = c.read<uint32_t>();
                        const auto end = c.read<uint16_t>();
                        e.preloadAt   = c.pos();
                        c.skip(e.preloadSize);
                        if (!c.ok() || end != kEntryEnd || e.length > kMaxEntrySize) {
                            why = "corrupt VPK directory entry";
                            return false;
                        }
                        m_entries.emplace(joinPath(dir, name, ext), e);
                    }
                }
            }
            if (!c.ok()) {
                why = "VPK directory truncated";
                return false;
            }
            return true;
        }

        // "maps/de_mirage" + "world_physics" + "vmdl_c"; a lone space = the root directory.
        [[nodiscard]] static std::string joinPath(std::string_view dir, std::string_view name, std::string_view ext) {
            std::string path;
            if (dir != " ") {
                path += dir;
                path += '/';
            }
            path += name;
            if (ext != " ") {
                path += '.';
                path += ext;
            }
            return path;
        }

        bool readBody(const Entry& e, std::vector<uint8_t>& out, std::string& why) const {
            if (e.archive == kThisArchive) {
                const size_t at = m_dataStart + e.offset;
                if (m_inMemory) {
                    const auto body = ByteView(m_bytes).sub(at, e.length);
                    if (!body) {
                        why = "VPK entry outside the file";
                        return false;
                    }
                    out.assign(body->data(), body->data() + body->size());
                    return true;
                }
                if (!readRange(m_file, at, e.length, out)) {
                    why = "VPK entry outside the file";
                    return false;
                }
                return true;
            }

            const auto archive = archivePath(e.archive);
            if (!archive) {
                why = "VPK entry in a missing archive";
                return false;
            }
            if (!readRange(*archive, e.offset, e.length, out)) {
                why = "can't read " + archive->string();
                return false;
            }
            return true;
        }

        // `x_dir.vpk` → `x_NNN.vpk`; nothing for single-file or in-memory VPKs.
        [[nodiscard]] std::optional<std::filesystem::path> archivePath(uint16_t archive) const {
            static constexpr std::string_view kDirSuffix = "_dir.vpk";
            const std::string name = m_file.filename().string();
            if (m_inMemory || name.size() <= kDirSuffix.size() || !name.ends_with(kDirSuffix))
                return std::nullopt;
            char number[16];
            std::snprintf(number, sizeof(number), "_%03u.vpk", static_cast<unsigned>(archive));
            return m_file.parent_path() / (name.substr(0, name.size() - kDirSuffix.size()) + number);
        }

        [[nodiscard]] static bool readRange(const std::filesystem::path& file, size_t offset, size_t length,
                                            std::vector<uint8_t>& out) {
            std::ifstream in(file, std::ios::binary);
            if (!in)
                return false;
            in.seekg(0, std::ios::end);
            const auto size = static_cast<size_t>(in.tellg());
            if (offset > size || size - offset < length)
                return false;
            out.resize(length);
            in.seekg(static_cast<std::streamoff>(offset));
            in.read(reinterpret_cast<char*>(out.data()), static_cast<std::streamsize>(length));
            return static_cast<size_t>(in.gcount()) == length;
        }
    };

} // namespace s2ghtline
