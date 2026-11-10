#pragma once

// Fixtures.h — writers for the formats the library reads, so the tests can
// build their input instead of needing the game:
//
//   Kv / encodeKv3()   a value tree -> binary KV3, versions 2-5, uncompressed,
//                      LZ4 (blobs in chained 16 KB frames) or zstd (raw-block
//                      frames: valid zstd, no compressor needed)
//   resourceFile()     blocks -> a compiled resource (`*_c`)
//   vpkFile()          files -> a VPK v2 (single file, or _dir + _000 archive)
//   physicsDoc()       a map's PHYS tree: hulls (both formats), meshes (both
//                      formats), a sphere, a capsule, three collision layers
//   surface files      surfaceproperties.vsurf_c + surfaceproperties_game.txt
//   FakeInstall        a game install on disk (official map, workshop item
//                      with a nested map VPK, pak01 with surface files)
//
// The KV3 writer mirrors Kv3.h's reader: every value goes to the stream the
// reader takes it from (types, 1 / 2 / 4 / 8-byte lanes, object lengths,
// strings, blobs), so a document round-trips through every code path.

#include "Bytes.h"
#include "Kv3.h"
#include "SurfaceNames.h"
#include "Vec3.h"

#include <lz4/lz4.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <random>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

namespace fixtures {

    using Bytes = std::vector<uint8_t>;

    template <class T>
    void put(Bytes& out, T v) {
        uint8_t b[sizeof(T)];
        std::memcpy(b, &v, sizeof(T));
        out.insert(out.end(), b, b + sizeof(T));
    }
    inline void append(Bytes& out, const Bytes& more) { out.insert(out.end(), more.begin(), more.end()); }
    inline void align(Bytes& out, size_t a) {
        while (out.size() % a)
            out.push_back(0);
    }
    inline Bytes bytesOf(std::string_view s) { return Bytes(s.begin(), s.end()); }

    // ── KV3 values ─────────────────────────────────────────────
    struct Kv {
        enum class Kind { Null, Bool, Int, UInt, Double, Float, String, Blob, Array, TypedArray, Object };
        // How a typed array is stored: count in the 4-byte lane, count in the
        // 1-byte lane, or count in the 1-byte lane with the elements in the
        // auxiliary buffer (v5's buffer 1).
        enum class Typed { Count4, Count1, Auxiliary };

        Kind                                    kind = Kind::Null;
        int64_t                                 i    = 0;
        uint64_t                                u    = 0;
        double                                  d    = 0.0;
        std::string                             s;
        Bytes                                   blob;
        std::vector<Kv>                         items;     // Array / TypedArray
        std::vector<std::pair<std::string, Kv>> members;   // Object
        Typed                                   typed = Typed::Count4;

        static Kv null() { return {}; }
        static Kv boolean(bool b) { Kv v; v.kind = Kind::Bool; v.i = b; return v; }
        static Kv integer(int64_t x) { Kv v; v.kind = Kind::Int; v.i = x; return v; }
        static Kv uinteger(uint64_t x) { Kv v; v.kind = Kind::UInt; v.u = x; return v; }
        static Kv real(double x) { Kv v; v.kind = Kind::Double; v.d = x; return v; }
        static Kv f32(float x) { Kv v; v.kind = Kind::Float; v.d = x; return v; }
        static Kv str(std::string x) { Kv v; v.kind = Kind::String; v.s = std::move(x); return v; }
        static Kv bytes(Bytes x) { Kv v; v.kind = Kind::Blob; v.blob = std::move(x); return v; }
        static Kv array(std::vector<Kv> x) { Kv v; v.kind = Kind::Array; v.items = std::move(x); return v; }
        static Kv typedArray(std::vector<Kv> x, Typed t = Typed::Count4) {
            Kv v; v.kind = Kind::TypedArray; v.items = std::move(x); v.typed = t; return v;
        }
        static Kv object(std::vector<std::pair<std::string, Kv>> x) {
            Kv v; v.kind = Kind::Object; v.members = std::move(x); return v;
        }
        // Three floats as a typed array (how vectors are stored).
        static Kv vec3(s2ghtline::Vec3 p, Typed t = Typed::Count4) {
            return typedArray({ f32(p.x), f32(p.y), f32(p.z) }, t);
        }
    };

    enum class Compression : uint32_t { None = 0, Lz4 = 1, Zstd = 2 };

    inline const char* toString(Compression c) {
        return c == Compression::None ? "none" : c == Compression::Lz4 ? "lz4" : "zstd";
    }

    // A zstd frame of raw (stored) blocks: valid input for any zstd decoder.
    inline Bytes zstdRawFrame(const uint8_t* data, size_t size) {
        Bytes out;
        put<uint32_t>(out, 0xFD2FB528u);   // magic
        out.push_back(0xA0);               // single segment, 4-byte content size
        put<uint32_t>(out, static_cast<uint32_t>(size));
        constexpr size_t kMaxBlock = 128 * 1024;
        size_t at = 0;
        do {
            const size_t   n      = std::min(kMaxBlock, size - at);
            const bool     last   = at + n == size;
            const uint32_t header = (last ? 1u : 0u) | (0u << 1) | (static_cast<uint32_t>(n) << 3);   // raw block
            out.push_back(static_cast<uint8_t>(header));
            out.push_back(static_cast<uint8_t>(header >> 8));
            out.push_back(static_cast<uint8_t>(header >> 16));
            out.insert(out.end(), data + at, data + at + n);
            at += n;
        } while (at < size);
        return out;
    }
    inline Bytes zstdRawFrame(const Bytes& b) { return zstdRawFrame(b.data(), b.size()); }

    inline Bytes lz4Block(const Bytes& b) {
        Bytes out(static_cast<size_t>(LZ4_compressBound(static_cast<int>(b.size()))));
        const int n = LZ4_compress_default(reinterpret_cast<const char*>(b.data()), reinterpret_cast<char*>(out.data()),
                                           static_cast<int>(b.size()), static_cast<int>(out.size()));
        if (n <= 0 && !b.empty())
            throw std::runtime_error("LZ4 compression failed");
        out.resize(static_cast<size_t>(n));
        return out;
    }

    // ── KV3 writer ─────────────────────────────────────────────
    class Kv3Writer {
    public:
        Kv3Writer(int version, Compression compression) : m_version(version), m_compression(compression) {}

        Bytes encode(const Kv& root) {
            const uint8_t code = compactCode(root);
            type(code);
            payload(root, code, m_main);
            return m_version >= 5 ? assembleV5() : assembleV4();
        }

    private:
        enum : uint8_t {
            kNull = 1, kBoolean = 2, kInt64 = 3, kUInt64 = 4, kDouble = 5, kString = 6, kBlob = 7,
            kArray = 8, kObject = 9, kArrayTyped = 10, kInt32 = 11, kUInt32 = 12, kTrue = 13, kFalse = 14,
            kInt64Zero = 15, kInt64One = 16, kDoubleZero = 17, kDoubleOne = 18, kFloat = 19,
            kInt16 = 20, kUInt16 = 21, kInt8 = 22, kUInt8 = 23, kArrayTypedByteLength = 24,
            kArrayTypedAuxiliary = 25,
        };
        static constexpr uint32_t kTrailer   = 0xFFEEDD00;
        static constexpr size_t   kLz4Frame  = 16384;

        struct Lanes {
            Bytes  b1, b2, b4, b8;
            size_t n2 = 0, n4 = 0, n8 = 0;
        };

        int         m_version;
        Compression m_compression;
        Bytes       m_types;
        Lanes       m_main, m_aux;
        std::vector<int32_t>                 m_objectLengths;   // v5
        std::vector<std::string>             m_strings;
        std::unordered_map<std::string, int> m_stringIds;
        std::vector<int32_t>                 m_blobLengths;
        Bytes                                m_blobData;

        Lanes& auxLanes() { return m_version >= 5 ? m_aux : m_main; }

        int32_t stringId(const std::string& s) {
            const auto [it, added] = m_stringIds.try_emplace(s, static_cast<int>(m_strings.size()));
            if (added)
                m_strings.push_back(s);
            return it->second;
        }

        // Strings carry a flag byte (0x80), as the game's files do for annotated strings.
        void type(uint8_t code) {
            if (code == kString) {
                m_types.push_back(static_cast<uint8_t>(code | 0x80));
                m_types.push_back(0);
            }
            else {
                m_types.push_back(code);
            }
        }

        // The smallest encoding of a single value.
        uint8_t compactCode(const Kv& v) const {
            switch (v.kind) {
            case Kv::Kind::Null:   return kNull;
            case Kv::Kind::Bool:   return v.i ? kTrue : kFalse;
            case Kv::Kind::Int:
                if (v.i == 0) return kInt64Zero;
                if (v.i == 1) return kInt64One;
                if (v.i >= INT8_MIN && v.i <= INT8_MAX) return kInt8;
                if (m_version >= 4 && v.i >= INT16_MIN && v.i <= INT16_MAX) return kInt16;
                if (v.i >= INT32_MIN && v.i <= INT32_MAX) return kInt32;
                return kInt64;
            case Kv::Kind::UInt:
                if (v.u <= UINT8_MAX) return kUInt8;
                if (m_version >= 4 && v.u <= UINT16_MAX) return kUInt16;
                if (v.u <= UINT32_MAX) return kUInt32;
                return kUInt64;
            case Kv::Kind::Double:
                if (v.d == 0.0) return kDoubleZero;
                if (v.d == 1.0) return kDoubleOne;
                return kDouble;
            case Kv::Kind::Float:  return kFloat;
            case Kv::Kind::String: return kString;
            case Kv::Kind::Blob:   return kBlob;
            case Kv::Kind::Array:  return kArray;
            case Kv::Kind::TypedArray:
                return v.typed == Kv::Typed::Count4 ? kArrayTyped
                     : v.typed == Kv::Typed::Count1 ? kArrayTypedByteLength : kArrayTypedAuxiliary;
            case Kv::Kind::Object: return kObject;
            }
            return kNull;
        }

        // One encoding for every element of a typed array.
        static uint8_t fixedCode(const Kv& v) {
            switch (v.kind) {
            case Kv::Kind::Bool:   return kBoolean;
            case Kv::Kind::Int:    return kInt32;
            case Kv::Kind::UInt:   return kUInt32;
            case Kv::Kind::Double: return kDouble;
            case Kv::Kind::Float:  return kFloat;
            case Kv::Kind::String: return kString;
            default:               throw std::logic_error("typed arrays hold scalars here");
            }
        }

        void payload(const Kv& v, uint8_t code, Lanes& lane) {
            switch (code) {
            case kNull: case kTrue: case kFalse: case kInt64Zero: case kInt64One: case kDoubleZero: case kDoubleOne:
                return;
            case kBoolean: lane.b1.push_back(v.i ? 1 : 0); return;
            case kInt8:    put<int8_t>(lane.b1, static_cast<int8_t>(v.i)); return;
            case kUInt8:   put<uint8_t>(lane.b1, static_cast<uint8_t>(v.u)); return;
            case kInt16:   put<int16_t>(lane.b2, static_cast<int16_t>(v.i)); ++lane.n2; return;
            case kUInt16:  put<uint16_t>(lane.b2, static_cast<uint16_t>(v.u)); ++lane.n2; return;
            case kInt32:   put<int32_t>(lane.b4, static_cast<int32_t>(v.i)); ++lane.n4; return;
            case kUInt32:  put<uint32_t>(lane.b4, static_cast<uint32_t>(v.u)); ++lane.n4; return;
            case kFloat:   put<float>(lane.b4, static_cast<float>(v.d)); ++lane.n4; return;
            case kInt64:   put<int64_t>(lane.b8, v.i); ++lane.n8; return;
            case kUInt64:  put<uint64_t>(lane.b8, v.u); ++lane.n8; return;
            case kDouble:  put<double>(lane.b8, v.d); ++lane.n8; return;
            case kString:  put<int32_t>(m_main.b4, stringId(v.s)); ++m_main.n4; return;
            case kBlob:
                m_blobLengths.push_back(static_cast<int32_t>(v.blob.size()));
                append(m_blobData, v.blob);
                return;
            case kArray:
                put<int32_t>(m_main.b4, static_cast<int32_t>(v.items.size()));
                ++m_main.n4;
                for (const Kv& e : v.items) {
                    const uint8_t c = compactCode(e);
                    type(c);
                    payload(e, c, m_main);
                }
                return;
            case kArrayTyped: case kArrayTypedByteLength: case kArrayTypedAuxiliary: {
                if (v.items.empty())
                    throw std::logic_error("typed arrays can't be empty (use an array)");
                if (code == kArrayTyped) {
                    put<int32_t>(m_main.b4, static_cast<int32_t>(v.items.size()));
                    ++m_main.n4;
                }
                else {
                    if (v.items.size() > 255)
                        throw std::logic_error("a byte-length typed array holds at most 255 elements");
                    m_main.b1.push_back(static_cast<uint8_t>(v.items.size()));
                }
                const uint8_t element = fixedCode(v.items[0]);
                type(element);
                Lanes& elementLane = code == kArrayTypedAuxiliary ? auxLanes() : m_main;
                for (const Kv& e : v.items)
                    payload(e, element, elementLane);
                return;
            }
            case kObject:
                if (m_version >= 5) {
                    m_objectLengths.push_back(static_cast<int32_t>(v.members.size()));
                }
                else {
                    put<int32_t>(m_main.b4, static_cast<int32_t>(v.members.size()));
                    ++m_main.n4;
                }
                for (const auto& [name, member] : v.members) {
                    const uint8_t c = compactCode(member);
                    type(c);
                    put<int32_t>(m_main.b4, stringId(name));
                    ++m_main.n4;
                    payload(member, c, m_main);
                }
                return;
            default:
                throw std::logic_error("unhandled KV3 code");
            }
        }

        // Lanes as the reader lays them: 1-byte, then each wider lane aligned
        // to its size; empty lanes take no space (and no alignment).
        static void layLanes(Bytes& out, const Bytes& b1, const Bytes& b2, const Bytes& b4, const Bytes& b8) {
            append(out, b1);
            if (!b2.empty()) { align(out, 2); append(out, b2); }
            if (!b4.empty()) { align(out, 4); append(out, b4); }
            if (!b8.empty()) { align(out, 8); append(out, b8); }
        }

        Bytes stringBytes() const {
            Bytes out;
            for (const auto& s : m_strings) {
                append(out, bytesOf(s));
                out.push_back(0);
            }
            return out;
        }

        // LZ4 blobs: each blob in frames of up to 16 KB, one chained stream.
        void lz4Blobs(Bytes& frames, std::vector<uint16_t>& frameSizes) const {
            LZ4_stream_t* stream = LZ4_createStream();
            const char*   base   = reinterpret_cast<const char*>(m_blobData.data());
            size_t at = 0;
            for (const int32_t length : m_blobLengths) {
                const size_t end = at + static_cast<size_t>(length);
                while (at < end) {
                    const int n = static_cast<int>(std::min(kLz4Frame, end - at));
                    Bytes dst(static_cast<size_t>(LZ4_compressBound(n)));
                    const int c = LZ4_compress_fast_continue(stream, base + at, reinterpret_cast<char*>(dst.data()), n,
                                                             static_cast<int>(dst.size()), 1);
                    if (c <= 0 || c > UINT16_MAX)
                        throw std::runtime_error("LZ4 frame compression failed");
                    frames.insert(frames.end(), dst.begin(), dst.begin() + c);
                    frameSizes.push_back(static_cast<uint16_t>(c));
                    at += static_cast<size_t>(n);
                }
            }
            LZ4_freeStream(stream);
        }

        // After the types: the trailer, or the blob lengths, the trailer and (LZ4) the frame sizes.
        void tail(Bytes& out, const std::vector<uint16_t>& frameSizes) const {
            if (m_blobLengths.empty()) {
                put<uint32_t>(out, kTrailer);
                return;
            }
            for (const int32_t l : m_blobLengths)
                put<int32_t>(out, l);
            put<uint32_t>(out, kTrailer);
            for (const uint16_t f : frameSizes)
                put<uint16_t>(out, f);
        }

        Bytes compress(const Bytes& b) const {
            switch (m_compression) {
            case Compression::None: return b;
            case Compression::Lz4:  return lz4Block(b);
            case Compression::Zstd: return zstdRawFrame(b);
            }
            return b;
        }

        void header(Bytes& out, int32_t bytes1, int32_t bytes2, int32_t bytes4, int32_t bytes8, int32_t typesSize,
                    int32_t uncompressedTotal, int32_t compressedTotal, size_t frameCount) const {
            put<uint32_t>(out, 0x4B563300u | static_cast<uint32_t>(m_version));
            for (int i = 0; i < 16; ++i)
                out.push_back(static_cast<uint8_t>(0x10 + i));   // format GUID (ignored)
            put<uint32_t>(out, static_cast<uint32_t>(m_compression));
            put<uint16_t>(out, 0);   // dictionary
            put<uint16_t>(out, static_cast<uint16_t>(m_compression == Compression::Lz4 ? kLz4Frame : 0));
            put<int32_t>(out, bytes1);
            put<int32_t>(out, bytes4);
            put<int32_t>(out, bytes8);
            put<int32_t>(out, typesSize);
            put<uint16_t>(out, 0);   // object count hint
            put<uint16_t>(out, 0);   // array count hint
            put<int32_t>(out, uncompressedTotal);
            put<int32_t>(out, compressedTotal);
            put<int32_t>(out, static_cast<int32_t>(m_blobLengths.size()));
            put<int32_t>(out, static_cast<int32_t>(m_blobData.size()));
            if (m_version >= 4) {
                put<int32_t>(out, bytes2);
                put<int32_t>(out, static_cast<int32_t>(frameCount * 2));
            }
        }

        Bytes assembleV4() {
            Bytes frames;
            std::vector<uint16_t> frameSizes;
            if (m_compression == Compression::Lz4)
                lz4Blobs(frames, frameSizes);

            Bytes b4;
            put<int32_t>(b4, static_cast<int32_t>(m_strings.size()));
            append(b4, m_main.b4);
            Bytes buffer;
            layLanes(buffer, m_main.b1, m_main.b2, b4, m_main.b8);
            if (m_main.b8.empty())
                align(buffer, 8);
            const Bytes strings = stringBytes();
            append(buffer, strings);
            append(buffer, m_types);
            tail(buffer, frameSizes);

            // zstd before v5: the buffer and the blobs in one run (here as two frames).
            Bytes body;
            if (m_compression == Compression::Zstd) {
                body = zstdRawFrame(buffer);
                if (!m_blobData.empty())
                    append(body, zstdRawFrame(m_blobData));
            }
            else {
                body = compress(buffer);
            }

            Bytes out;
            header(out, static_cast<int32_t>(m_main.b1.size()), static_cast<int32_t>(m_main.n2),
                   static_cast<int32_t>(1 + m_main.n4), static_cast<int32_t>(m_main.n8),
                   static_cast<int32_t>(strings.size() + m_types.size()), static_cast<int32_t>(buffer.size()),
                   static_cast<int32_t>(body.size()), frameSizes.size());
            append(out, body);
            if (!m_blobLengths.empty()) {
                if (m_compression == Compression::None)
                    append(out, m_blobData);
                else if (m_compression == Compression::Lz4)
                    append(out, frames);
                put<uint32_t>(out, kTrailer);
            }
            return out;
        }

        Bytes assembleV5() {
            Bytes frames;
            std::vector<uint16_t> frameSizes;
            if (m_compression == Compression::Lz4)
                lz4Blobs(frames, frameSizes);

            // Buffer 1: the strings (in the 1-byte lane) and the auxiliary lanes.
            Bytes b1 = stringBytes();
            append(b1, m_aux.b1);
            Bytes b4;
            put<int32_t>(b4, static_cast<int32_t>(m_strings.size()));
            append(b4, m_aux.b4);
            Bytes buffer1;
            layLanes(buffer1, b1, m_aux.b2, b4, m_aux.b8);

            // Buffer 2: object lengths, the main lanes, the types, the tail.
            Bytes buffer2;
            for (const int32_t n : m_objectLengths)
                put<int32_t>(buffer2, n);
            layLanes(buffer2, m_main.b1, m_main.b2, m_main.b4, m_main.b8);
            append(buffer2, m_types);
            tail(buffer2, frameSizes);

            const Bytes c1 = compress(buffer1), c2 = compress(buffer2);
            Bytes blobs;
            if (!m_blobLengths.empty())
                blobs = m_compression == Compression::None ? m_blobData
                      : m_compression == Compression::Lz4  ? frames
                                                           : zstdRawFrame(m_blobData);

            Bytes out;
            header(out, static_cast<int32_t>(b1.size()), static_cast<int32_t>(m_aux.n2),
                   static_cast<int32_t>(1 + m_aux.n4), static_cast<int32_t>(m_aux.n8),
                   static_cast<int32_t>(m_types.size()),
                   static_cast<int32_t>(buffer1.size() + buffer2.size() + m_blobData.size()),
                   static_cast<int32_t>(c1.size() + c2.size() + blobs.size()), frameSizes.size());
            put<int32_t>(out, static_cast<int32_t>(buffer1.size()));
            put<int32_t>(out, static_cast<int32_t>(c1.size()));
            put<int32_t>(out, static_cast<int32_t>(buffer2.size()));
            put<int32_t>(out, static_cast<int32_t>(c2.size()));
            put<int32_t>(out, static_cast<int32_t>(m_main.b1.size()));
            put<int32_t>(out, static_cast<int32_t>(m_main.n2));
            put<int32_t>(out, static_cast<int32_t>(m_main.n4));
            put<int32_t>(out, static_cast<int32_t>(m_main.n8));
            put<int32_t>(out, 0);   // node count hint
            put<int32_t>(out, static_cast<int32_t>(m_objectLengths.size()));
            put<int32_t>(out, 0);   // array count hint
            put<int32_t>(out, 0);   // array element count hint
            append(out, c1);
            append(out, c2);
            if (!m_blobLengths.empty()) {
                append(out, blobs);
                put<uint32_t>(out, kTrailer);
            }
            return out;
        }
    };

    inline Bytes encodeKv3(const Kv& root, int version, Compression compression) {
        return Kv3Writer(version, compression).encode(root);
    }

    // ── Compiled resource (`*_c`) ──────────────────────────────
    inline Bytes resourceFile(const std::vector<std::pair<uint32_t, Bytes>>& blocks) {
        const size_t tableStart = 16, entrySize = 12;
        Bytes out;
        put<uint32_t>(out, 0);    // file size, patched below
        put<uint16_t>(out, 12);   // header version
        put<uint16_t>(out, 0);    // type version
        put<uint32_t>(out, 8);    // block table: 8 bytes after this field
        put<uint32_t>(out, static_cast<uint32_t>(blocks.size()));
        size_t data = tableStart + entrySize * blocks.size();
        for (size_t i = 0; i < blocks.size(); ++i) {
            const size_t entry = tableStart + entrySize * i;
            put<uint32_t>(out, blocks[i].first);
            put<uint32_t>(out, static_cast<uint32_t>(data - (entry + 4)));
            put<uint32_t>(out, static_cast<uint32_t>(blocks[i].second.size()));
            data += blocks[i].second.size();
        }
        for (const auto& [tag, bytes] : blocks)
            append(out, bytes);
        const auto size = static_cast<uint32_t>(out.size());
        std::memcpy(out.data(), &size, 4);
        return out;
    }

    // ── VPK v2 ─────────────────────────────────────────────────
    struct VpkOptions {
        bool   archive = false;   // data in a separate `_000` archive (the dir file holds the tree only)
        size_t preload = 0;       // bytes of each file stored in the tree
    };
    struct VpkBytes {
        Bytes dir;       // the .vpk (or _dir.vpk)
        Bytes archive;   // the _000.vpk (VpkOptions::archive)
    };

    inline VpkBytes vpkFile(const std::map<std::string, Bytes>& files, VpkOptions options = {}) {
        // ext -> dir -> name -> file
        std::map<std::string, std::map<std::string, std::map<std::string, const Bytes*>>> tree;
        for (const auto& [path, bytes] : files) {
            const size_t slash = path.rfind('/');
            const std::string dir  = slash == std::string::npos ? " " : path.substr(0, slash);
            std::string       file = slash == std::string::npos ? path : path.substr(slash + 1);
            const size_t dot = file.rfind('.');
            const std::string ext = dot == std::string::npos ? " " : file.substr(dot + 1);
            if (dot != std::string::npos)
                file.resize(dot);
            tree[ext][dir][file] = &bytes;
        }

        Bytes treeBytes, data;
        const auto cstr = [&](const std::string& s) {
            append(treeBytes, bytesOf(s));
            treeBytes.push_back(0);
        };
        for (const auto& [ext, dirs] : tree) {
            cstr(ext);
            for (const auto& [dir, names] : dirs) {
                cstr(dir);
                for (const auto& [name, bytes] : names) {
                    cstr(name);
                    const size_t preload = std::min(options.preload, bytes->size());
                    put<uint32_t>(treeBytes, 0);   // crc (not checked)
                    put<uint16_t>(treeBytes, static_cast<uint16_t>(preload));
                    put<uint16_t>(treeBytes, options.archive ? 0 : 0x7FFF);
                    put<uint32_t>(treeBytes, static_cast<uint32_t>(data.size()));
                    put<uint32_t>(treeBytes, static_cast<uint32_t>(bytes->size() - preload));
                    put<uint16_t>(treeBytes, 0xFFFF);
                    treeBytes.insert(treeBytes.end(), bytes->begin(), bytes->begin() + static_cast<ptrdiff_t>(preload));
                    data.insert(data.end(), bytes->begin() + static_cast<ptrdiff_t>(preload), bytes->end());
                }
                cstr("");
            }
            cstr("");
        }
        cstr("");

        VpkBytes out;
        put<uint32_t>(out.dir, 0x55AA1234);
        put<uint32_t>(out.dir, 2);
        put<uint32_t>(out.dir, static_cast<uint32_t>(treeBytes.size()));
        put<uint32_t>(out.dir, options.archive ? 0 : static_cast<uint32_t>(data.size()));
        put<uint32_t>(out.dir, 0);   // archive MD5 section
        put<uint32_t>(out.dir, 0);   // other MD5 section
        put<uint32_t>(out.dir, 0);   // signature section
        append(out.dir, treeBytes);
        if (options.archive)
            out.archive = std::move(data);
        else
            append(out.dir, data);
        return out;
    }

    // ── A map's physics ────────────────────────────────────────
    struct Box {
        s2ghtline::Vec3 min, max;
        [[nodiscard]] s2ghtline::Vec3 corner(int i) const {
            return { i & 1 ? max.x : min.x, i & 2 ? max.y : min.y, i & 4 ? max.z : min.z };
        }
    };

    // A box as a convex hull: 8 corners, 6 quads wound counter-clockwise seen
    // from outside, 24 half-edges.  `blobs`: the current format (blobs), else
    // the older one (arrays of values / objects).
    inline Kv hullShape(const Box& b, int layer, int surface, bool blobs) {
        static constexpr int kFaces[6][4] = {
            { 0, 4, 6, 2 }, { 1, 3, 7, 5 },   // -x, +x
            { 0, 1, 5, 4 }, { 2, 6, 7, 3 },   // -y, +y
            { 0, 2, 3, 1 }, { 4, 5, 7, 6 },   // -z, +z
        };
        struct HalfEdge { uint8_t next, twin, origin, face; };
        std::array<HalfEdge, 24> edges{};
        for (int f = 0; f < 6; ++f)
            for (int k = 0; k < 4; ++k)
                edges[f * 4 + k] = { static_cast<uint8_t>(f * 4 + (k + 1) % 4), 0, static_cast<uint8_t>(kFaces[f][k]),
                                     static_cast<uint8_t>(f) };
        for (int e = 0; e < 24; ++e) {   // twin: the edge running the other way
            const uint8_t from = edges[e].origin, to = edges[edges[e].next].origin;
            for (int o = 0; o < 24; ++o)
                if (edges[o].origin == to && edges[edges[o].next].origin == from)
                    edges[e].twin = static_cast<uint8_t>(o);
        }

        Kv hull;
        if (blobs) {
            Bytes positions, edgeBytes, faces;
            for (int i = 0; i < 8; ++i) {
                const auto p = b.corner(i);
                put(positions, p.x); put(positions, p.y); put(positions, p.z);
            }
            for (const auto& e : edges) {
                edgeBytes.push_back(e.next); edgeBytes.push_back(e.twin);
                edgeBytes.push_back(e.origin); edgeBytes.push_back(e.face);
            }
            for (int f = 0; f < 6; ++f)
                faces.push_back(static_cast<uint8_t>(f * 4));
            hull = Kv::object({ { "m_VertexPositions", Kv::bytes(positions) },
                                { "m_Vertices", Kv::bytes(Bytes(8, 0)) },   // per-vertex edge indices (unused)
                                { "m_Edges", Kv::bytes(edgeBytes) },
                                { "m_Faces", Kv::bytes(faces) } });
        }
        else {
            std::vector<Kv> vertices, edgeList, faceList;
            for (int i = 0; i < 8; ++i)
                vertices.push_back(Kv::vec3(b.corner(i), Kv::Typed::Count1));
            for (const auto& e : edges)
                edgeList.push_back(Kv::object({ { "m_nNext", Kv::integer(e.next) }, { "m_nTwin", Kv::integer(e.twin) },
                                                { "m_nOrigin", Kv::integer(e.origin) }, { "m_nFace", Kv::integer(e.face) } }));
            for (int f = 0; f < 6; ++f)
                faceList.push_back(Kv::object({ { "m_nEdge", Kv::integer(f * 4) } }));
            hull = Kv::object({ { "m_Vertices", Kv::array(vertices) },
                                { "m_Edges", Kv::array(edgeList) },
                                { "m_Faces", Kv::array(faceList) } });
        }
        return Kv::object({ { "m_nCollisionAttributeIndex", Kv::integer(layer) },
                            { "m_nSurfacePropertyIndex", Kv::integer(surface) },
                            { "m_Hull", hull } });
    }

    // A flat quad (z = `z`, facing up) as a two-triangle mesh, with a
    // per-triangle material each.
    inline Kv meshShape(float x0, float y0, float x1, float y1, float z, int layer, int surface,
                        std::array<int, 2> materials, bool blobs) {
        const s2ghtline::Vec3 v[4] = { { x0, y0, z }, { x1, y0, z }, { x1, y1, z }, { x0, y1, z } };
        const int tris[2][3] = { { 0, 1, 2 }, { 0, 2, 3 } };
        Kv mesh;
        if (blobs) {
            Bytes vertices, triangles, mats;
            for (const auto& p : v) { put(vertices, p.x); put(vertices, p.y); put(vertices, p.z); }
            for (const auto& t : tris) for (int k : t) put<int32_t>(triangles, k);
            for (int m : materials) mats.push_back(static_cast<uint8_t>(m));
            mesh = Kv::object({ { "m_Vertices", Kv::bytes(vertices) },
                                { "m_Triangles", Kv::bytes(triangles) },
                                { "m_Materials", Kv::bytes(mats) } });
        }
        else {
            std::vector<Kv> vertices, triangles, mats;
            for (const auto& p : v) vertices.push_back(Kv::vec3(p, Kv::Typed::Auxiliary));
            for (const auto& t : tris)
                triangles.push_back(Kv::object({ { "m_nIndex", Kv::typedArray({ Kv::integer(t[0]), Kv::integer(t[1]),
                                                                                  Kv::integer(t[2]) }) } }));
            for (int m : materials) mats.push_back(Kv::integer(m));
            mesh = Kv::object({ { "m_Vertices", Kv::array(vertices) },
                                { "m_Triangles", Kv::array(triangles) },
                                { "m_Materials", Kv::array(mats) } });
        }
        return Kv::object({ { "m_nCollisionAttributeIndex", Kv::integer(layer) },
                            { "m_nSurfacePropertyIndex", Kv::integer(surface) },
                            { "m_Mesh", mesh } });
    }

    inline Kv layerAttributes(std::vector<std::string> interactAs, std::vector<std::string> exclude = {}) {
        std::vector<Kv> as, ex;
        for (auto& s : interactAs) as.push_back(Kv::str(std::move(s)));
        for (auto& s : exclude) ex.push_back(Kv::str(std::move(s)));
        return Kv::object({ { "m_CollisionGroupString", Kv::str("default") },
                            { "m_InteractAsStrings", Kv::array(std::move(as)) },
                            { "m_InteractExcludeStrings", Kv::array(std::move(ex)) } });
    }

    // The test map ("box world"):
    //   layer 0 solid, 1 passbullets (a fence), 2 playerclip
    //   surfaces 0 "concrete", 1 "metal", 2 "glass"
    //   hull A: a 100-unit solid cube at the origin (blob format, concrete)
    //   hull B: a thin fence slab at x = 200 (array format, layer 1, metal)
    //   mesh 1: a floor quad at z = -10 (blobs; materials concrete / metal)
    //   mesh 2: a clip quad at z = 300 (arrays; materials glass / glass)
    //   a sphere (r 8) and a capsule (r 4) on the solid layer
    struct PhysicsExpect {
        static constexpr uint32_t hulls = 2, meshes = 2, spheres = 1, capsules = 1;
        static constexpr uint32_t hullTris = 24, meshTris = 4;
        static constexpr uint32_t primitiveTris = 60 + 80;   // sphere: 3 rings, capsule: 4; 10 sides
        static constexpr size_t   tris = hullTris + meshTris + primitiveTris;
        static constexpr size_t   layers = 3, surfaces = 3;
    };
    inline const Box kSolidCube{ { 0.f, 0.f, 0.f }, { 100.f, 100.f, 100.f } };
    inline const Box kFence{ { 200.f, -50.f, 0.f }, { 202.f, 150.f, 100.f } };

    inline Kv physicsDoc() {
        using s2ghtline::surfaceHash;
        const Kv shape = Kv::object({
            { "m_hulls", Kv::array({ hullShape(kSolidCube, 0, 0, true), hullShape(kFence, 1, 1, false) }) },
            { "m_meshes", Kv::array({ meshShape(-500.f, -500.f, 500.f, 500.f, -10.f, 0, 0, { 0, 1 }, true),
                                      meshShape(-500.f, -500.f, 500.f, 500.f, 300.f, 2, 2, { 2, 2 }, false) }) },
            { "m_spheres", Kv::array({ Kv::object({
                { "m_nCollisionAttributeIndex", Kv::integer(0) }, { "m_nSurfacePropertyIndex", Kv::integer(0) },
                { "m_Sphere", Kv::object({ { "m_vCenter", Kv::vec3({ 400.f, 0.f, 50.f }) },
                                           { "m_flRadius", Kv::f32(8.f) } }) } }) }) },
            { "m_capsules", Kv::array({ Kv::object({
                { "m_nCollisionAttributeIndex", Kv::integer(0) }, { "m_nSurfacePropertyIndex", Kv::integer(1) },
                { "m_Capsule", Kv::object({ { "m_vCenter", Kv::array({ Kv::vec3({ 400.f, 100.f, 0.f }),
                                                                       Kv::vec3({ 400.f, 100.f, 80.f }) }) },
                                            { "m_flRadius", Kv::real(4.0) } }) } }) }) },
        });
        return Kv::object({
            { "m_surfacePropertyHashes", Kv::typedArray({ Kv::uinteger(surfaceHash("concrete")),
                                                          Kv::uinteger(surfaceHash("metal")),
                                                          Kv::uinteger(surfaceHash("glass")) }) },
            { "m_collisionAttributes", Kv::array({ layerAttributes({ "solid" }),
                                                   layerAttributes({ "passbullets" }, { "player" }),
                                                   layerAttributes({ "playerclip" }) }) },
            { "m_bindPose", Kv::array({}) },
            { "m_parts", Kv::array({ Kv::object({ { "m_rnShape", shape } }) }) },
        });
    }

    inline Bytes physicsModelFile(int version = 5, Compression c = Compression::Zstd) {
        return resourceFile({ { s2ghtline::fourCC("DATA"), bytesOf("unused") },
                              { s2ghtline::fourCC("PHYS"), encodeKv3(physicsDoc(), version, c) } });
    }

    // ── Surface properties ─────────────────────────────────────
    // vsurf_c: each surface's name hash and its base's.  The game text sets
    // a few values; the rest is inherited (metalgrate <- metal <- default).
    inline Bytes surfaceVsurfFile() {
        using s2ghtline::surfaceHash;
        const auto entry = [](const char* name, const char* base) {
            return Kv::object({ { "surfacePropertyName", Kv::str(name) },
                                { "m_nameHash", Kv::uinteger(surfaceHash(name)) },
                                { "m_baseNameHash", Kv::uinteger(base ? surfaceHash(base) : 0u) } });
        };
        const Kv doc = Kv::object({ { "SurfacePropertiesList", Kv::array({
            entry("default", nullptr), entry("concrete", "default"), entry("metal", "default"),
            entry("metalgrate", "metal"), entry("glass", "default") }) } });
        return resourceFile({ { s2ghtline::fourCC("DATA"), encodeKv3(doc, 5, Compression::Lz4) } });
    }

    inline const char* kSurfaceGameText = R"(<!-- kv3 encoding:text:version{e21c7f3c-8a33-41c5-9977-a76d3a32aa0d} format:generic:version{7412167c-06e9-4698-aff2-e63eb59037e7} -->
{
	// comments and other keys are skipped
	version = 2
	SurfacePropertiesList =
	[
		{
			surfacePropertyName = "default"
			gamematerial = "C"
			bulletPenetrationDistanceModifier = 0.5
			bulletPenetrationDamageModifier = 0.5
			nested = { ignored = [ 1, 2, 3 ] }
		},
		{
			surfacePropertyName = "concrete"
			bulletPenetrationDistanceModifier = 0.25
		},
		{
			surfacePropertyName = "metal"
			gamematerial = "M"
			bulletPenetrationDistanceModifier = 0.4
			bulletPenetrationDamageModifier = 0.3
		},
		{
			surfacePropertyName = "metalgrate"
			gamematerial = "G"
		},
		{
			surfacePropertyName = "glass"
			gamematerial = "Y"
			bulletPenetrationDistanceModifier = 0.99
		},
	]
}
)";

    // ── A game install on disk ─────────────────────────────────
    // <root>/steamapps/common/Counter-Strike Global Offensive/game/csgo:
    //   maps/box_world.vpk                         official map (single-file VPK)
    //   pak01_dir.vpk + pak01_000.vpk              surface files (archive VPK, preloads)
    // <root>/steamapps/workshop/content/730/123456789/
    //   123456789_dir.vpk + _000.vpk               a workshop item nesting maps/ws_box.vpk
    class FakeInstall {
    public:
        static constexpr const char* kWorkshopId = "123456789";

        explicit FakeInstall(const std::string& name) {
            std::random_device rd;
            m_root = std::filesystem::temp_directory_path() / (name + "_" + std::to_string(rd()));
            m_csgo = m_root / "steamapps" / "common" / "Counter-Strike Global Offensive" / "game" / "csgo";
            std::filesystem::create_directories(m_csgo / "maps");

            writeMap("box_world", physicsModelFile(5, Compression::Zstd));

            const VpkBytes pak = vpkFile({ { "surfaceproperties/surfaceproperties.vsurf_c", surfaceVsurfFile() },
                                           { "scripts/surfaceproperties_game.txt", bytesOf(kSurfaceGameText) } },
                                         { .archive = true, .preload = 16 });
            write(m_csgo / "pak01_dir.vpk", pak.dir);
            write(m_csgo / "pak01_000.vpk", pak.archive);

            // The workshop item: its VPK holds the map's VPK (LZ4 physics, as workshop maps have).
            const VpkBytes nested = vpkFile({ { "maps/ws_box/world_physics.vmdl_c", physicsModelFile(4, Compression::Lz4) } });
            const VpkBytes item   = vpkFile({ { "maps/ws_box.vpk", nested.dir },
                                              { "maps/ws_box_3dsky.vpk", vpkFile({}).dir } },
                                            { .archive = true });
            const auto itemDir = workshopDir() / kWorkshopId;
            std::filesystem::create_directories(itemDir);
            write(itemDir / (std::string(kWorkshopId) + "_dir.vpk"), item.dir);
            write(itemDir / (std::string(kWorkshopId) + "_000.vpk"), item.archive);
        }
        ~FakeInstall() {
            std::error_code ec;
            std::filesystem::remove_all(m_root, ec);
        }
        FakeInstall(const FakeInstall&)            = delete;
        FakeInstall& operator=(const FakeInstall&) = delete;

        [[nodiscard]] const std::filesystem::path& csgoDir() const { return m_csgo; }
        [[nodiscard]] std::filesystem::path workshopDir() const {
            return m_root / "steamapps" / "workshop" / "content" / "730";
        }

        // An official map's VPK: maps/<map>.vpk holding maps/<map>/world_physics.vmdl_c.
        void writeMap(const std::string& map, const Bytes& physics) const {
            write(m_csgo / "maps" / (map + ".vpk"), vpkFile({ { "maps/" + map + "/world_physics.vmdl_c", physics } }).dir);
        }

        static void write(const std::filesystem::path& p, const Bytes& bytes) {
            std::ofstream out(p, std::ios::binary | std::ios::trunc);
            out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        }

    private:
        std::filesystem::path m_root, m_csgo;
    };

} // namespace fixtures
