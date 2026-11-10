#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <string_view>
#include <type_traits>

// Bytes.h — bounds-checked little-endian reads over bytes read from the
// game's files.  Map files are untrusted input: every offset and length in
// them is checked before use, so a corrupt or unexpected file fails cleanly
// instead of reading out of bounds.
namespace s2ghtline {

    // A FourCC tag ("PHYS") as the integer the files store, built at compile time.
    consteval uint32_t fourCC(const char (&tag)[5]) {
        return  static_cast<uint32_t>(static_cast<uint8_t>(tag[0]))        |
               (static_cast<uint32_t>(static_cast<uint8_t>(tag[1])) << 8)  |
               (static_cast<uint32_t>(static_cast<uint8_t>(tag[2])) << 16) |
               (static_cast<uint32_t>(static_cast<uint8_t>(tag[3])) << 24);
    }

    // ByteView ---------------------------------------------------
    // A non-owning window onto bytes; every read is checked.
    class ByteView {
    public:
        ByteView() = default;
        explicit ByteView(std::span<const uint8_t> bytes) : m_bytes(bytes) {}

        [[nodiscard]] size_t                   size()  const { return m_bytes.size(); }
        [[nodiscard]] bool                     empty() const { return m_bytes.empty(); }
        [[nodiscard]] const uint8_t*           data()  const { return m_bytes.data(); }
        [[nodiscard]] std::span<const uint8_t> span()  const { return m_bytes; }

        [[nodiscard]] bool has(size_t offset, size_t length) const {
            return offset <= m_bytes.size() && m_bytes.size() - offset >= length;
        }

        template <class T>
        [[nodiscard]] std::optional<T> at(size_t offset) const {
            static_assert(std::is_trivially_copyable_v<T>);
            if (!has(offset, sizeof(T)))
                return std::nullopt;
            T value;
            std::memcpy(&value, m_bytes.data() + offset, sizeof(T));
            return value;
        }

        [[nodiscard]] std::optional<ByteView> sub(size_t offset, size_t length) const {
            if (!has(offset, length))
                return std::nullopt;
            return ByteView(m_bytes.subspan(offset, length));
        }

        // A null-terminated string starting at offset (the terminator must be in bounds).
        [[nodiscard]] std::optional<std::string_view> cstr(size_t offset) const {
            if (offset >= m_bytes.size())
                return std::nullopt;
            const auto* start = reinterpret_cast<const char*>(m_bytes.data() + offset);
            const auto* end   = static_cast<const char*>(std::memchr(start, 0, m_bytes.size() - offset));
            if (!end)
                return std::nullopt;
            return std::string_view(start, static_cast<size_t>(end - start));
        }

    private:
        std::span<const uint8_t> m_bytes;
    };

    // ByteCursor ---------------------------------------------------
    // Sequential reads with a sticky failure flag: a read past the end returns
    // a zero value and marks the cursor failed, so a parser reads a run of
    // fields and checks ok() once.
    class ByteCursor {
    public:
        ByteCursor() = default;
        explicit ByteCursor(ByteView view) : m_view(view) {}

        [[nodiscard]] bool   ok()        const { return m_ok; }
        [[nodiscard]] size_t pos()       const { return m_pos; }
        [[nodiscard]] size_t remaining() const { return m_ok ? m_view.size() - m_pos : 0; }

        template <class T>
        T read() {
            if (auto v = m_ok ? m_view.at<T>(m_pos) : std::nullopt) {
                m_pos += sizeof(T);
                return *v;
            }
            m_ok = false;
            return T{};
        }

        ByteView take(size_t length) {
            if (auto v = m_ok ? m_view.sub(m_pos, length) : std::nullopt) {
                m_pos += length;
                return *v;
            }
            m_ok = false;
            return {};
        }

        std::string_view cstr() {
            if (auto s = m_ok ? m_view.cstr(m_pos) : std::nullopt) {
                m_pos += s->size() + 1;
                return *s;
            }
            m_ok = false;
            return {};
        }

        void skip(size_t length) { take(length); }

        void align(size_t alignment) {
            const size_t aligned = (m_pos + alignment - 1) / alignment * alignment;
            skip(aligned - m_pos);
        }

        void fail() { m_ok = false; }

    private:
        ByteView m_view;
        size_t   m_pos = 0;
        bool     m_ok  = true;
    };

} // namespace s2ghtline
