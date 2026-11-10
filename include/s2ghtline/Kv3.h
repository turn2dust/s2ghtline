#pragma once

#include "Bytes.h"

#include <lz4/lz4.h>
#include <zstd/zstd.h>

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

// Kv3.h — decoder for binary KeyValues3, the format of Source 2's
// compiled resource data (a map's PHYS block).  Versions 2-5 (current CS2:
// official maps v5 + zstd, workshop maps v4 / v5 + LZ4); v1 and the legacy
// "VKV3" encoding fail as unsupported.  Format reference: ValveResourceFormat
// (BinaryKV3.cs); tools/MapOracle checks this decoder against it.
//
// Encoding: values are split by size into "lanes" (1, 2, 4, 8-byte values),
// a byte stream of types, a string table, per-object member counts and
// binary blobs; decoding walks the types and pulls each value from its lane.
//
// The decoded tree is flat: nodes in one vector, containers referring to a
// run of child indices.  Object keys are kept only as hashes, and lookups use
// `"m_name"_kv` (hashed at compile time): a lookup compares integers, and the
// decoded tree stores no key strings.
namespace s2ghtline {

    // FNV-1a, the hash of an object key.
    [[nodiscard]] constexpr uint32_t kv3Hash(std::string_view s) {
        uint32_t h = 2166136261u;
        for (char ch : s) {
            h ^= static_cast<uint8_t>(ch);
            h *= 16777619u;
        }
        return h;
    }

    namespace kv3_literals {
        consteval uint32_t operator""_kv(const char* s, size_t n) { return kv3Hash(std::string_view(s, n)); }
    }

    enum class Kv3Type : uint8_t { Null, Bool, Int, UInt, Double, String, Blob, Array, Object };

    class Kv3Doc;

    // Kv3Value ---------------------------------------------------
    // A handle to one value of a document (valid while the document lives).
    // Missing members / wrong types give an invalid value or nullopt, never a crash.
    class Kv3Value {
    public:
        Kv3Value() = default;

        [[nodiscard]] bool    valid()    const { return m_doc != nullptr; }
        [[nodiscard]] Kv3Type type()     const;   // Null when invalid
        [[nodiscard]] bool    isArray()  const { return type() == Kv3Type::Array; }
        [[nodiscard]] bool    isObject() const { return type() == Kv3Type::Object; }

        // Object member by key hash ("m_parts"_kv); invalid if missing.
        [[nodiscard]] Kv3Value operator[](uint32_t key) const;
        // Array / object element count, and element by position.
        [[nodiscard]] size_t   size() const;
        [[nodiscard]] Kv3Value at(size_t i) const;

        [[nodiscard]] std::optional<int64_t>          integer() const;   // bool / int / uint
        [[nodiscard]] std::optional<double>           number()  const;   // double / int / uint
        [[nodiscard]] std::optional<std::string_view> string()  const;
        [[nodiscard]] std::optional<ByteView>         blob()    const;

    private:
        friend class Kv3Doc;
        Kv3Value(const Kv3Doc* doc, uint32_t node) : m_doc(doc), m_node(node) {}

        const Kv3Doc* m_doc  = nullptr;
        uint32_t      m_node = 0;
    };

    // Kv3Doc ---------------------------------------------------
    class Kv3Doc {
    public:
        // Decode a binary KV3 block (e.g. a resource's PHYS block).
        [[nodiscard]] static std::optional<Kv3Doc> parse(ByteView block, std::string& why);

        [[nodiscard]] Kv3Value root()      const { return m_nodes.empty() ? Kv3Value() : Kv3Value(this, 0); }
        [[nodiscard]] size_t   nodeCount() const { return m_nodes.size(); }

    private:
        friend class Kv3Value;
        friend class Kv3Decoder;

        static constexpr uint32_t kNoString = 0xFFFFFFFFu;

        struct Node {
            Kv3Type  type  = Kv3Type::Null;
            uint32_t key   = 0;   // member key hash (0 for array elements / the root)
            union {
                int64_t  i;
                uint64_t u;
                double   d;
            }        value{ 0 };
            uint32_t first = 0;   // Array / Object: first child slot; String: string index; Blob: offset
            uint32_t count = 0;   // Array / Object: child count; Blob: length
        };

        std::vector<Node>        m_nodes;
        std::vector<uint32_t>    m_children;   // node indices, each container's run contiguous
        std::vector<std::string> m_strings;
        std::vector<uint8_t>     m_blobs;
    };

    // Kv3Decoder ---------------------------------------------------
    // One decode: the header, the decompressed buffers and their lanes.
    class Kv3Decoder {
    public:
        Kv3Decoder(ByteView block, std::string& why) : m_in(block), m_why(why) {}

        std::optional<Kv3Doc> run() {
            if (!readHeader() || !readBuffer1() || !readBuffer2() || !readBlobs())
                return std::nullopt;

            const Kv3BinaryType rootType = readType();
            m_doc.m_nodes.reserve(1024);
            readValue(rootType, m_main, 0, 0);
            if (failed())
                return fail("corrupt KV3 data");
            return std::move(m_doc);
        }

    private:
        // The file's value types (BinaryKV3.NodeType.cs).
        enum Kv3BinaryType : uint8_t {
            kNull = 1, kBoolean = 2, kInt64 = 3, kUInt64 = 4, kDouble = 5, kString = 6, kBlob = 7,
            kArray = 8, kObject = 9, kArrayTyped = 10, kInt32 = 11, kUInt32 = 12, kTrue = 13, kFalse = 14,
            kInt64Zero = 15, kInt64One = 16, kDoubleZero = 17, kDoubleOne = 18, kFloat = 19,
            kInt16 = 20, kUInt16 = 21, kInt8 = 22, kUInt8 = 23, kArrayTypedByteLength = 24,
            kArrayTypedAuxiliary = 25,
        };

        enum class Compression : uint32_t { None = 0, Lz4 = 1, Zstd = 2 };

        struct Lanes {
            ByteCursor b1, b2, b4, b8;
            [[nodiscard]] bool ok() const { return b1.ok() && b2.ok() && b4.ok() && b8.ok(); }
        };

        static constexpr uint32_t kMagicBase     = 0x4B563300;   // "\0" "3VK" + version in the low byte
        static constexpr uint32_t kTrailer       = 0xFFEEDD00;
        static constexpr uint16_t kLz4FrameSize  = 16384;
        static constexpr int32_t  kMaxSize       = 1 << 30;
        static constexpr size_t   kMaxNodes      = 16u << 20;
        static constexpr int      kMaxDepth      = 64;

        ByteCursor   m_in;
        std::string& m_why;
        Kv3Doc       m_doc;
        bool         m_failed = false;

        // Header
        int         m_version     = 0;
        Compression m_compression = Compression::None;
        uint16_t    m_dictionary  = 0, m_frameSize = 0;
        int32_t     m_bytes1 = 0, m_bytes2 = 0, m_bytes4 = 0, m_bytes8 = 0, m_typesSize = 0;
        int32_t     m_uncompressedTotal = 0, m_compressedTotal = 0, m_blobCount = 0, m_blobBytes = 0;
        int32_t     m_uncompressed1 = 0, m_compressed1 = 0, m_uncompressed2 = 0, m_compressed2 = 0;
        int32_t     m_bytes1b = 0, m_bytes2b = 0, m_bytes4b = 0, m_bytes8b = 0, m_objects2 = 0;

        // Decompressed data and where each part lies in it
        std::vector<uint8_t> m_buffer1, m_buffer2;
        Lanes      m_main;            // lanes of the buffer holding the values
        Lanes      m_aux;             // v5: buffer 1's lanes (typed-auxiliary arrays); before v5 unused
        Lanes*     m_auxLanes = &m_main;
        ByteCursor m_types, m_objectLengths, m_blobLengths, m_blobSizes;
        size_t     m_blobPos = 0;

        std::nullopt_t fail(const char* why) {
            m_why    = why;
            m_failed = true;
            return std::nullopt;
        }

        [[nodiscard]] bool failed() const {
            return m_failed || !m_main.ok() || !m_aux.ok() || !m_types.ok() || !m_objectLengths.ok() ||
                   !m_blobLengths.ok();
        }

        // ── Header ───────────────────────────────────────────
        bool readHeader() {
            const auto magic = m_in.read<uint32_t>();
            m_version = static_cast<int>(magic & 0xFF);
            if (!m_in.ok() || (magic & 0xFFFFFF00) != kMagicBase || m_version < 2 || m_version > 5) {
                fail("unsupported KV3 encoding");
                return false;
            }
            m_in.skip(16);   // format GUID
            m_compression = static_cast<Compression>(m_in.read<uint32_t>());
            m_dictionary  = m_in.read<uint16_t>();
            m_frameSize   = m_in.read<uint16_t>();
            m_bytes1      = m_in.read<int32_t>();
            m_bytes4      = m_in.read<int32_t>();
            m_bytes8      = m_in.read<int32_t>();
            m_typesSize   = m_in.read<int32_t>();
            m_in.read<uint16_t>();   // object count (allocation hint)
            m_in.read<uint16_t>();   // array count (allocation hint)
            m_uncompressedTotal = m_in.read<int32_t>();
            m_compressedTotal   = m_in.read<int32_t>();
            m_blobCount         = m_in.read<int32_t>();
            m_blobBytes         = m_in.read<int32_t>();
            if (m_version >= 4) {
                m_bytes2 = m_in.read<int32_t>();
                m_in.read<int32_t>();   // size of the block compressed sizes (LZ4 frame table)
            }
            if (m_version >= 5) {
                m_uncompressed1 = m_in.read<int32_t>();
                m_compressed1   = m_in.read<int32_t>();
                m_uncompressed2 = m_in.read<int32_t>();
                m_compressed2   = m_in.read<int32_t>();
                m_bytes1b       = m_in.read<int32_t>();
                m_bytes2b       = m_in.read<int32_t>();
                m_bytes4b       = m_in.read<int32_t>();
                m_bytes8b       = m_in.read<int32_t>();
                m_in.read<int32_t>();   // node count (hint)
                m_objects2      = m_in.read<int32_t>();
                m_in.read<int32_t>();   // array count (hint)
                m_in.read<int32_t>();   // array element count (hint)
            }
            else {
                m_uncompressed1 = m_uncompressedTotal;
                m_compressed1   = m_compressedTotal;
            }

            const int32_t sizes[] = { m_bytes1, m_bytes2, m_bytes4, m_bytes8, m_typesSize, m_uncompressedTotal,
                                      m_compressedTotal, m_blobCount, m_blobBytes, m_uncompressed1, m_compressed1,
                                      m_uncompressed2, m_compressed2, m_bytes1b, m_bytes2b, m_bytes4b, m_bytes8b,
                                      m_objects2 };
            for (int32_t s : sizes) {
                if (s < 0 || s > kMaxSize) {
                    fail("corrupt KV3 header");
                    return false;
                }
            }
            if (!m_in.ok() || m_compression > Compression::Zstd || m_dictionary != 0) {
                fail("unsupported KV3 compression");
                return false;
            }
            if (m_compression == Compression::Lz4 && m_frameSize != kLz4FrameSize) {
                fail("unsupported KV3 LZ4 frame size");
                return false;
            }
            return true;
        }

        // ── Decompression ────────────────────────────────────
        // `compressed` bytes of input → exactly `size` bytes (zstd may hold several frames).
        bool decompress(int32_t compressed, size_t size, std::vector<uint8_t>& out) {
            out.resize(size);
            if (m_compression == Compression::None) {
                const ByteView raw = m_in.take(size);
                if (m_in.ok() && size)
                    std::memcpy(out.data(), raw.data(), size);
                return m_in.ok();
            }

            const ByteView src = m_in.take(static_cast<size_t>(compressed));
            if (!m_in.ok())
                return false;
            if (m_compression == Compression::Lz4) {
                const int n = LZ4_decompress_safe(reinterpret_cast<const char*>(src.data()),
                                                  reinterpret_cast<char*>(out.data()),
                                                  compressed, static_cast<int>(size));
                return n >= 0 && static_cast<size_t>(n) == size;
            }
            const size_t n = ZSTD_decompress(out.data(), size, src.data(), src.size());
            return !ZSTD_isError(n) && n == size;
        }

        // Buffer lanes: 1-byte, then each wider lane aligned to its size (empty lanes are skipped).
        static Lanes layLanes(ByteCursor& c, int32_t n1, int32_t n2, int32_t n4, int32_t n8) {
            Lanes l;
            if (n1 > 0) { l.b1 = ByteCursor(c.take(static_cast<size_t>(n1))); }
            if (n2 > 0) { c.align(2); l.b2 = ByteCursor(c.take(static_cast<size_t>(n2) * 2)); }
            if (n4 > 0) { c.align(4); l.b4 = ByteCursor(c.take(static_cast<size_t>(n4) * 4)); }
            if (n8 > 0) { c.align(8); l.b8 = ByteCursor(c.take(static_cast<size_t>(n8) * 8)); }
            return l;
        }

        bool readStrings(ByteCursor& from, int32_t count) {
            if (count < 0 || static_cast<size_t>(count) > from.remaining()) {
                fail("corrupt KV3 string table");
                return false;
            }
            m_doc.m_strings.reserve(static_cast<size_t>(count));
            for (int32_t i = 0; i < count && from.ok(); ++i)
                m_doc.m_strings.emplace_back(from.cstr());
            if (!from.ok()) {
                fail("corrupt KV3 string table");
                return false;
            }
            return true;
        }

        // Buffer 1: the lanes, string count + strings; before v5 also the types and blob sizes.
        bool readBuffer1() {
            // Before v5, zstd stores this buffer and the blobs as consecutive frames in one run.
            const bool blobsInside = m_version < 5 && m_compression == Compression::Zstd;
            const size_t size = static_cast<size_t>(m_uncompressed1) + (blobsInside ? m_blobBytes : 0);
            if (!decompress(m_compressed1, size, m_buffer1)) {
                fail("KV3 buffer failed to decompress");
                return false;
            }
            if (m_bytes4 <= 0) {
                fail("corrupt KV3 header");
                return false;
            }

            const auto view = ByteView(m_buffer1).sub(0, static_cast<size_t>(m_uncompressed1));
            ByteCursor c(view.value_or(ByteView{}));
            Lanes lanes = layLanes(c, m_bytes1, m_bytes2, m_bytes4, m_bytes8);
            if (m_bytes8 == 0 && m_version < 5)
                c.align(8);
            const int32_t stringCount = lanes.b4.read<int32_t>();

            if (m_version >= 5) {
                if (!readStrings(lanes.b1, stringCount))
                    return false;
                m_aux      = lanes;
                m_auxLanes = &m_aux;
                return c.ok();
            }

            m_main = lanes;
            const size_t stringsStart = c.pos();
            if (!readStrings(c, stringCount))
                return false;
            const size_t stringBytes = c.pos() - stringsStart;
            if (static_cast<size_t>(m_typesSize) < stringBytes) {
                fail("corrupt KV3 types");
                return false;
            }
            m_types = ByteCursor(c.take(static_cast<size_t>(m_typesSize) - stringBytes));
            return readTail(c);
        }

        // Buffer 2 (v5): object member counts, the lanes, the types and blob sizes.
        bool readBuffer2() {
            if (m_version < 5)
                return true;
            if (!decompress(m_compressed2, static_cast<size_t>(m_uncompressed2), m_buffer2)) {
                fail("KV3 buffer failed to decompress");
                return false;
            }
            ByteCursor c{ ByteView(m_buffer2) };
            m_objectLengths = ByteCursor(c.take(static_cast<size_t>(m_objects2) * 4));
            m_main  = layLanes(c, m_bytes1b, m_bytes2b, m_bytes4b, m_bytes8b);
            m_types = ByteCursor(c.take(static_cast<size_t>(m_typesSize)));
            return readTail(c);
        }

        // After the types: the trailer, or (with blobs) the blob sizes.
        bool readTail(ByteCursor& c) {
            if (m_blobCount == 0) {
                if (c.read<uint32_t>() != kTrailer || !c.ok()) {
                    fail("corrupt KV3 buffer trailer");
                    return false;
                }
                return true;
            }
            m_blobSizes = ByteCursor(c.take(c.remaining()));
            if (!c.ok()) {
                fail("corrupt KV3 buffer");
                return false;
            }
            return true;
        }

        // ── Binary blobs ─────────────────────────────────────
        bool readBlobs() {
            if (m_blobCount == 0)
                return true;

            m_blobLengths = ByteCursor(m_blobSizes.take(static_cast<size_t>(m_blobCount) * 4));
            if (m_blobSizes.read<uint32_t>() != kTrailer || !m_blobSizes.ok()) {
                fail("corrupt KV3 blob table");
                return false;
            }

            auto& blobs = m_doc.m_blobs;
            bool ok = true;
            if (m_compression == Compression::None) {
                ok = decompress(0, static_cast<size_t>(m_blobBytes), blobs);
            }
            else if (m_compression == Compression::Lz4) {
                ok = readLz4Blobs(blobs);
            }
            else if (m_version >= 5) {
                const int32_t compressed = m_compressedTotal - m_compressed1 - m_compressed2;
                ok = compressed >= 0 && decompress(compressed, static_cast<size_t>(m_blobBytes), blobs);
            }
            else {
                // Decompressed with buffer 1.
                const auto* start = m_buffer1.data() + m_uncompressed1;
                blobs.assign(start, start + m_blobBytes);
            }
            if (!ok || m_in.read<uint32_t>() != kTrailer || !m_in.ok()) {
                fail("KV3 blobs failed to decompress");
                return false;
            }
            return true;
        }

        // LZ4 blobs: each blob in frames of up to 16 KB; frames are chained (a
        // frame may reference earlier output, also of earlier blobs), so the
        // output is decoded contiguously.
        bool readLz4Blobs(std::vector<uint8_t>& blobs) {
            blobs.resize(static_cast<size_t>(m_blobBytes));
            LZ4_streamDecode_t stream;
            LZ4_setStreamDecode(&stream, nullptr, 0);

            ByteCursor lengths = m_blobLengths;
            size_t at = 0;
            for (int32_t b = 0; b < m_blobCount; ++b) {
                const int32_t length = lengths.read<int32_t>();
                if (!lengths.ok() || length < 0 || static_cast<size_t>(length) > blobs.size() - at)
                    return false;
                const size_t end = at + static_cast<size_t>(length);
                while (at < end) {
                    const uint16_t frame = m_blobSizes.read<uint16_t>();
                    const ByteView src   = m_in.take(frame);
                    if (!m_blobSizes.ok() || !m_in.ok())
                        return false;
                    const int n = LZ4_decompress_safe_continue(
                        &stream, reinterpret_cast<const char*>(src.data()),
                        reinterpret_cast<char*>(blobs.data() + at), frame, static_cast<int>(end - at));
                    if (n < 1)
                        return false;
                    at += static_cast<size_t>(n);
                }
            }
            return at == blobs.size();
        }

        // ── Values ───────────────────────────────────────────
        // A type byte; from v3 0x80 = a flag byte follows (string / object
        // annotations, unused here), 0x40 = one more byte to skip.
        Kv3BinaryType readType() {
            uint8_t b = m_types.read<uint8_t>();
            if (m_version >= 3) {
                if (b & 0x80) m_types.read<uint8_t>();
                if (b & 0x40) m_types.read<uint8_t>();
                b &= 0x3F;
            }
            else {
                if (b & 0x80) m_types.read<uint8_t>();
                b &= 0x7F;
            }
            return static_cast<Kv3BinaryType>(b);
        }

        uint32_t addNode(Kv3Type type, uint32_t key) {
            if (m_doc.m_nodes.size() >= kMaxNodes) {
                fail("KV3 document too large");
                return 0;
            }
            Kv3Doc::Node n;
            n.type = type;
            n.key  = key;
            m_doc.m_nodes.push_back(n);
            return static_cast<uint32_t>(m_doc.m_nodes.size() - 1);
        }

        Kv3Doc::Node& node(uint32_t i) { return m_doc.m_nodes[i]; }

        // Reserve a container's child slots; false if the count can't be right.
        bool reserveChildren(uint32_t container, int32_t count) {
            if (count < 0 || static_cast<size_t>(count) > kMaxNodes - m_doc.m_nodes.size()) {
                fail("corrupt KV3 container size");
                return false;
            }
            node(container).first = static_cast<uint32_t>(m_doc.m_children.size());
            node(container).count = static_cast<uint32_t>(count);
            m_doc.m_children.resize(m_doc.m_children.size() + static_cast<size_t>(count));
            return true;
        }

        void setChild(uint32_t container, uint32_t i, uint32_t child) {
            m_doc.m_children[node(container).first + i] = child;
        }

        uint32_t stringIndex(int32_t id) const {
            return id >= 0 && static_cast<size_t>(id) < m_doc.m_strings.size() ? static_cast<uint32_t>(id)
                                                                                 : Kv3Doc::kNoString;
        }

        // One value of `type`: primitives from `lane` (the auxiliary buffer for
        // auxiliary typed arrays), strings / blobs / container sizes from the main buffer.
        uint32_t readValue(Kv3BinaryType type, Lanes& lane, uint32_t key, int depth) {
            if (failed())
                return 0;
            if (depth > kMaxDepth) {
                fail("KV3 nesting too deep");
                return 0;
            }

            switch (type) {
            case kNull:       return addNode(Kv3Type::Null, key);
            case kTrue:       return addInt(Kv3Type::Bool, key, 1);
            case kFalse:      return addInt(Kv3Type::Bool, key, 0);
            case kInt64Zero:  return addInt(Kv3Type::Int, key, 0);
            case kInt64One:   return addInt(Kv3Type::Int, key, 1);
            case kDoubleZero: return addDouble(key, 0.0);
            case kDoubleOne:  return addDouble(key, 1.0);

            case kBoolean:    return addInt(Kv3Type::Bool, key, lane.b1.read<uint8_t>() != 0);
            case kInt8:       return addInt(Kv3Type::Int,  key, lane.b1.read<int8_t>());
            case kUInt8:      return addInt(Kv3Type::UInt, key, lane.b1.read<uint8_t>());
            case kInt16:      return addInt(Kv3Type::Int,  key, lane.b2.read<int16_t>());
            case kUInt16:     return addInt(Kv3Type::UInt, key, lane.b2.read<uint16_t>());
            case kInt32:      return addInt(Kv3Type::Int,  key, lane.b4.read<int32_t>());
            case kUInt32:     return addInt(Kv3Type::UInt, key, lane.b4.read<uint32_t>());
            case kFloat:      return addDouble(key, lane.b4.read<float>());
            case kInt64:      return addInt(Kv3Type::Int,  key, lane.b8.read<int64_t>());
            case kUInt64:     return addUInt(key, lane.b8.read<uint64_t>());
            case kDouble:     return addDouble(key, lane.b8.read<double>());

            case kString: {
                const uint32_t n = addNode(Kv3Type::String, key);
                node(n).first = stringIndex(m_main.b4.read<int32_t>());
                return n;
            }
            case kBlob: {
                const int32_t length = m_blobLengths.read<int32_t>();
                if (length < 0 || static_cast<size_t>(length) > m_doc.m_blobs.size() - m_blobPos) {
                    fail("corrupt KV3 blob");
                    return 0;
                }
                const uint32_t n = addNode(Kv3Type::Blob, key);
                node(n).first = static_cast<uint32_t>(m_blobPos);
                node(n).count = static_cast<uint32_t>(length);
                m_blobPos += static_cast<size_t>(length);
                return n;
            }
            case kArray: {
                const uint32_t n = addNode(Kv3Type::Array, key);
                if (!reserveChildren(n, m_main.b4.read<int32_t>()))
                    return 0;
                for (uint32_t i = 0; i < node(n).count && !failed(); ++i)
                    setChild(n, i, readValue(readType(), m_main, 0, depth + 1));
                return n;
            }
            case kArrayTyped:
            case kArrayTypedByteLength:
            case kArrayTypedAuxiliary: {
                const int32_t count = type == kArrayTyped ? m_main.b4.read<int32_t>()
                                                          : static_cast<int32_t>(m_main.b1.read<uint8_t>());
                const Kv3BinaryType elementType = readType();
                Lanes& elementLane = type == kArrayTypedAuxiliary ? *m_auxLanes : m_main;
                const uint32_t n = addNode(Kv3Type::Array, key);
                if (count == 0 || !reserveChildren(n, count)) {
                    fail("corrupt KV3 typed array");
                    return 0;
                }
                for (uint32_t i = 0; i < node(n).count && !failed(); ++i)
                    setChild(n, i, readValue(elementType, elementLane, 0, depth + 1));
                return n;
            }
            case kObject: {
                const int32_t count = m_version >= 5 ? m_objectLengths.read<int32_t>() : m_main.b4.read<int32_t>();
                const uint32_t n = addNode(Kv3Type::Object, key);
                if (!reserveChildren(n, count))
                    return 0;
                for (uint32_t i = 0; i < node(n).count && !failed(); ++i) {
                    const Kv3BinaryType memberType = readType();
                    const uint32_t name = stringIndex(m_main.b4.read<int32_t>());
                    const uint32_t memberKey = name == Kv3Doc::kNoString ? 0 : kv3Hash(m_doc.m_strings[name]);
                    setChild(n, i, readValue(memberType, m_main, memberKey, depth + 1));
                }
                return n;
            }
            default:
                fail("unknown KV3 value type");
                return 0;
            }
        }

        uint32_t addInt(Kv3Type type, uint32_t key, int64_t v) {
            const uint32_t n = addNode(type, key);
            node(n).value.i = v;
            return n;
        }
        uint32_t addUInt(uint32_t key, uint64_t v) {
            const uint32_t n = addNode(Kv3Type::UInt, key);
            node(n).value.u = v;
            return n;
        }
        uint32_t addDouble(uint32_t key, double v) {
            const uint32_t n = addNode(Kv3Type::Double, key);
            node(n).value.d = v;
            return n;
        }
    };

    // ── Kv3Doc / Kv3Value definitions ────────────────────────
    inline std::optional<Kv3Doc> Kv3Doc::parse(ByteView block, std::string& why) {
        return Kv3Decoder(block, why).run();
    }

    inline Kv3Type Kv3Value::type() const { return valid() ? m_doc->m_nodes[m_node].type : Kv3Type::Null; }

    inline Kv3Value Kv3Value::operator[](uint32_t key) const {
        if (!isObject())
            return {};
        const auto& n = m_doc->m_nodes[m_node];
        for (uint32_t i = 0; i < n.count; ++i) {
            const uint32_t child = m_doc->m_children[n.first + i];
            if (m_doc->m_nodes[child].key == key)
                return Kv3Value(m_doc, child);
        }
        return {};
    }

    inline size_t Kv3Value::size() const {
        return isArray() || isObject() ? m_doc->m_nodes[m_node].count : 0;
    }

    inline Kv3Value Kv3Value::at(size_t i) const {
        if (i >= size())
            return {};
        return Kv3Value(m_doc, m_doc->m_children[m_doc->m_nodes[m_node].first + i]);
    }

    inline std::optional<int64_t> Kv3Value::integer() const {
        if (!valid())
            return std::nullopt;
        const auto& n = m_doc->m_nodes[m_node];
        switch (n.type) {
        case Kv3Type::Bool:
        case Kv3Type::Int:  return n.value.i;
        case Kv3Type::UInt: return n.value.u <= static_cast<uint64_t>(INT64_MAX)
                                   ? std::optional<int64_t>(static_cast<int64_t>(n.value.u)) : std::nullopt;
        default:            return std::nullopt;
        }
    }

    inline std::optional<double> Kv3Value::number() const {
        if (!valid())
            return std::nullopt;
        const auto& n = m_doc->m_nodes[m_node];
        switch (n.type) {
        case Kv3Type::Double: return n.value.d;
        case Kv3Type::Int:    return static_cast<double>(n.value.i);
        case Kv3Type::UInt:   return static_cast<double>(n.value.u);
        default:              return std::nullopt;
        }
    }

    inline std::optional<std::string_view> Kv3Value::string() const {
        if (!valid() || type() != Kv3Type::String)
            return std::nullopt;
        const uint32_t s = m_doc->m_nodes[m_node].first;
        return s == Kv3Doc::kNoString ? std::string_view() : std::string_view(m_doc->m_strings[s]);
    }

    inline std::optional<ByteView> Kv3Value::blob() const {
        if (!valid() || type() != Kv3Type::Blob)
            return std::nullopt;
        const auto& n = m_doc->m_nodes[m_node];
        return ByteView(m_doc->m_blobs).sub(n.first, n.count);
    }

} // namespace s2ghtline
