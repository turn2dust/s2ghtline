#pragma once

// SurfaceProperties.h — each surface property's bullet penetration
// values, from the game's files in pak01:
//
//   surfaceproperties/surfaceproperties.vsurf_c   every surface: its name hash
//                                                  (m_nameHash, = surfaceHash)
//                                                  and its base's (m_baseNameHash)
//   scripts/surfaceproperties_game.txt             text KV3: per surface name its
//                                                  gamematerial and
//                                                  bulletPenetration{Distance,
//                                                  Damage}Modifier
//
// A field a surface doesn't set comes from its base, its base's base, ...,
// then "default".  `type` packs the gamematerial string's first two bytes
// ("C" = 'C', "12" = '1' | '2' << 8), as the game's surface table stores it.
// Keys are matched by hash ("name"_kv), never by string.

#include "Bytes.h"
#include "Kv3.h"
#include "Resource.h"
#include "SurfaceNames.h"

#include <cctype>
#include <cmath>
#include <charconv>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace s2ghtline {

    struct SurfaceProps {
        float    penetration = 0.5f;   // bulletPenetrationDistanceModifier
        float    damage      = 0.5f;   // bulletPenetrationDamageModifier
        uint16_t type        = 'C';    // gamematerial
    };

    [[nodiscard]] constexpr uint16_t gameMaterialType(std::string_view s) {
        return s.empty() ? 0 : static_cast<uint16_t>(static_cast<uint8_t>(s[0]) | (s.size() > 1 ? static_cast<uint8_t>(s[1]) << 8 : 0));
    }

    namespace surface_detail {

        // The text KV3 file's game entries: each {...} element of the
        // SurfacePropertiesList array, its flat `key = value` pairs (key
        // hashed, value as text; nested blocks skipped).
        struct TextEntry {
            std::unordered_map<uint32_t, std::string> fields;
        };

        class TextReader {
        public:
            explicit TextReader(std::string_view text) : m_s(text) {}

            // Every element of the root's `listKey` array.
            bool readList(uint32_t listKey, std::vector<TextEntry>& out, std::string& why) {
                skipHeader();
                if (!expect('{'))
                    return fail(why, "no root object");
                for (;;) {
                    skipSpace();
                    if (m_i >= m_s.size())
                        return fail(why, "unterminated root object");
                    if (m_s[m_i] == '}')
                        return true;
                    const auto key = readKey();
                    if (!key || !expect('='))
                        return fail(why, "bad key");
                    if (kv3Hash(*key) == listKey) {
                        if (!readEntries(out))
                            return fail(why, "bad SurfacePropertiesList");
                    }
                    else if (!skipValue()) {
                        return fail(why, "bad value");
                    }
                }
            }

        private:
            std::string_view m_s;
            size_t           m_i = 0;

            static bool fail(std::string& why, const char* what) {
                why = std::string("surfaceproperties_game.txt: ") + what;
                return false;
            }
            void skipHeader() {   // <!-- kv3 ... -->
                skipSpace();
                if (m_s.substr(m_i, 4) == "<!--") {
                    const size_t end = m_s.find("-->", m_i);
                    m_i = end == std::string_view::npos ? m_s.size() : end + 3;
                }
            }
            void skipSpace() {
                for (;;) {
                    while (m_i < m_s.size() && std::isspace(static_cast<unsigned char>(m_s[m_i])))
                        ++m_i;
                    if (m_s.substr(m_i, 2) == "//") {   // line comment
                        const size_t end = m_s.find('\n', m_i);
                        m_i = end == std::string_view::npos ? m_s.size() : end + 1;
                        continue;
                    }
                    return;
                }
            }
            bool expect(char c) {
                skipSpace();
                if (m_i < m_s.size() && m_s[m_i] == c) {
                    ++m_i;
                    return true;
                }
                return false;
            }
            std::optional<std::string_view> readKey() {
                skipSpace();
                const size_t start = m_i;
                while (m_i < m_s.size() && (std::isalnum(static_cast<unsigned char>(m_s[m_i])) || m_s[m_i] == '_' || m_s[m_i] == '.'))
                    ++m_i;
                if (m_i == start)
                    return std::nullopt;
                return m_s.substr(start, m_i - start);
            }
            // A scalar: "string" or a bare token (number, true / false).
            std::optional<std::string> readScalar() {
                skipSpace();
                if (m_i >= m_s.size())
                    return std::nullopt;
                if (m_s[m_i] == '"') {
                    const size_t end = m_s.find('"', m_i + 1);
                    if (end == std::string_view::npos)
                        return std::nullopt;
                    std::string v(m_s.substr(m_i + 1, end - m_i - 1));
                    m_i = end + 1;
                    return v;
                }
                const size_t start = m_i;
                while (m_i < m_s.size() && !std::isspace(static_cast<unsigned char>(m_s[m_i])) && m_s[m_i] != ',' &&
                       m_s[m_i] != '}' && m_s[m_i] != ']')
                    ++m_i;
                if (m_i == start)
                    return std::nullopt;
                return std::string(m_s.substr(start, m_i - start));
            }
            // Any value: {...} / [...] skipped by nesting (strings honoured).
            bool skipValue() {
                skipSpace();
                if (m_i >= m_s.size())
                    return false;
                if (m_s[m_i] != '{' && m_s[m_i] != '[')
                    return readScalar().has_value();
                int depth = 0;
                for (; m_i < m_s.size(); ++m_i) {
                    const char c = m_s[m_i];
                    if (c == '"') {
                        const size_t end = m_s.find('"', m_i + 1);
                        if (end == std::string_view::npos)
                            return false;
                        m_i = end;
                    }
                    else if (c == '{' || c == '[') {
                        ++depth;
                    }
                    else if ((c == '}' || c == ']') && --depth == 0) {
                        ++m_i;
                        return true;
                    }
                }
                return false;
            }
            // [ {..}, {..} ]
            bool readEntries(std::vector<TextEntry>& out) {
                if (!expect('['))
                    return false;
                for (;;) {
                    skipSpace();
                    if (m_i >= m_s.size())
                        return false;
                    if (m_s[m_i] == ']') {
                        ++m_i;
                        return true;
                    }
                    if (m_s[m_i] == ',') {
                        ++m_i;
                        continue;
                    }
                    if (!expect('{'))
                        return false;
                    TextEntry e;
                    for (;;) {
                        skipSpace();
                        if (m_i >= m_s.size())
                            return false;
                        if (m_s[m_i] == '}') {
                            ++m_i;
                            break;
                        }
                        if (m_s[m_i] == ',') {
                            ++m_i;
                            continue;
                        }
                        const auto key = readKey();
                        if (!key || !expect('='))
                            return false;
                        skipSpace();
                        if (m_i < m_s.size() && (m_s[m_i] == '{' || m_s[m_i] == '[')) {
                            if (!skipValue())
                                return false;
                            continue;
                        }
                        const auto value = readScalar();
                        if (!value)
                            return false;
                        e.fields[kv3Hash(*key)] = *value;
                    }
                    out.push_back(std::move(e));
                }
            }
        };

        [[nodiscard]] inline std::optional<float> toFloat(const std::string& s) {
            float v = 0.f;
            const auto [p, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
            return ec == std::errc() && p == s.data() + s.size() && std::isfinite(v) ? std::optional<float>(v) : std::nullopt;
        }

    } // namespace surface_detail

    // SurfaceTable ---------------------------------------------------
    class SurfaceTable {
    public:
        // From the two files' bytes.  Nothing (and `why`) when either is unreadable.
        [[nodiscard]] static std::optional<SurfaceTable> parse(ByteView vsurf, std::string_view gameText, std::string& why) {
            using namespace kv3_literals;
            const auto data = findBlock(vsurf, fourCC("DATA"));
            const auto doc  = data ? Kv3Doc::parse(*data, why) : std::nullopt;
            if (!doc) {
                if (why.empty())
                    why = "surfaceproperties.vsurf_c: no data block";
                return std::nullopt;
            }
            // Name hash -> base name hash (and its name).
            std::unordered_map<uint32_t, uint32_t> baseOf;
            SurfaceTable t;
            const Kv3Value list = doc->root()["SurfacePropertiesList"_kv];
            for (size_t i = 0; i < list.size(); ++i) {
                const Kv3Value e    = list.at(i);
                const auto     name = e["m_nameHash"_kv].integer();
                if (!name)
                    continue;
                baseOf[static_cast<uint32_t>(*name)] = static_cast<uint32_t>(e["m_baseNameHash"_kv].integer().value_or(0));
                if (const auto n = e["surfacePropertyName"_kv].string())
                    t.m_names[static_cast<uint32_t>(*name)] = std::string(*n);
            }
            if (baseOf.empty()) {
                why = "surfaceproperties.vsurf_c: no surfaces";
                return std::nullopt;
            }

            // The game values each surface sets itself, by name hash.
            std::vector<surface_detail::TextEntry> entries;
            if (!surface_detail::TextReader(gameText).readList("SurfacePropertiesList"_kv, entries, why))
                return std::nullopt;
            struct Own {
                std::optional<float>    penetration, damage;
                std::optional<uint16_t> type;
            };
            std::unordered_map<uint32_t, Own> own;
            for (const auto& e : entries) {
                const auto name = e.fields.find("surfacePropertyName"_kv);
                if (name == e.fields.end())
                    continue;
                Own o;
                if (const auto f = e.fields.find("bulletPenetrationDistanceModifier"_kv); f != e.fields.end())
                    o.penetration = surface_detail::toFloat(f->second);
                if (const auto f = e.fields.find("bulletPenetrationDamageModifier"_kv); f != e.fields.end())
                    o.damage = surface_detail::toFloat(f->second);
                if (const auto f = e.fields.find("gamematerial"_kv); f != e.fields.end())
                    o.type = gameMaterialType(f->second);
                own[surfaceHash(name->second)] = o;
            }
            const uint32_t kDefault = surfaceHash("default");
            if (!own.count(kDefault)) {
                why = "surfaceproperties_game.txt: no \"default\" surface";
                return std::nullopt;
            }

            // Resolve: self, base, base's base, ..., then default.
            const auto resolve = [&](uint32_t hash) {
                SurfaceProps p;
                bool havePen = false, haveDmg = false, haveType = false;
                uint32_t h = hash;
                for (int depth = 0; depth < 32 && !(havePen && haveDmg && haveType); ++depth) {
                    if (const auto it = own.find(h); it != own.end()) {
                        if (!havePen && it->second.penetration)  { p.penetration = *it->second.penetration; havePen = true; }
                        if (!haveDmg && it->second.damage)       { p.damage = *it->second.damage; haveDmg = true; }
                        if (!haveType && it->second.type)        { p.type = *it->second.type; haveType = true; }
                    }
                    if (h == kDefault)
                        break;
                    const auto base = baseOf.find(h);
                    h = base != baseOf.end() && base->second ? base->second : kDefault;
                }
                return p;
            };
            for (const auto& [hash, base] : baseOf)
                t.m_props[hash] = resolve(hash);
            t.m_default = resolve(kDefault);
            return t;
        }

        // A surface's values; an unknown hash gets "default"'s.
        [[nodiscard]] const SurfaceProps& get(uint32_t nameHash) const {
            const auto it = m_props.find(nameHash);
            return it != m_props.end() ? it->second : m_default;
        }
        [[nodiscard]] bool   knows(uint32_t nameHash) const { return m_props.count(nameHash) != 0; }
        // A surface's name ("" if unknown).
        [[nodiscard]] const std::string& name(uint32_t nameHash) const {
            static const std::string none;
            const auto it = m_names.find(nameHash);
            return it != m_names.end() ? it->second : none;
        }
        [[nodiscard]] size_t size() const { return m_props.size(); }
        [[nodiscard]] const SurfaceProps& defaults() const { return m_default; }

    private:
        std::unordered_map<uint32_t, SurfaceProps> m_props;
        std::unordered_map<uint32_t, std::string>  m_names;
        SurfaceProps                               m_default;
    };

} // namespace s2ghtline
