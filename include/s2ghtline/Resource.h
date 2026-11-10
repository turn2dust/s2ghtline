#pragma once

#include "Bytes.h"

#include <cstdint>
#include <optional>

// Resource.h — the container of Source 2's compiled resources (`*_c`):
//   u32 file size, u16 header version, u16 type version,
//   u32 block table offset (from its own field), u32 block count,
//   then per block {FourCC tag, u32 offset (from its own field), u32 size}.
namespace s2ghtline {

    // Every block with `tag`, in file order: f(bytes) returns false to stop.
    // False if the block table is unreadable.
    template <class F>
    bool forEachBlock(ByteView file, uint32_t tag, F&& f) {
        constexpr size_t   kTableField = 8;    // where the block table offset is stored
        constexpr size_t   kBlockEntry = 12;
        constexpr uint32_t kMaxBlocks  = 64;

        const auto tableOffset = file.at<uint32_t>(kTableField);
        const auto blockCount  = file.at<uint32_t>(kTableField + 4);
        if (!tableOffset || !blockCount || *blockCount > kMaxBlocks)
            return false;

        const size_t table = kTableField + *tableOffset;
        for (uint32_t i = 0; i < *blockCount; ++i) {
            const size_t entry = table + i * kBlockEntry;
            const auto blockTag = file.at<uint32_t>(entry);
            const auto offset   = file.at<uint32_t>(entry + 4);
            const auto size     = file.at<uint32_t>(entry + 8);
            if (!blockTag || !offset || !size)
                return false;
            if (*blockTag != tag)
                continue;
            if (const auto bytes = file.sub(entry + 4 + *offset, *size); bytes && !f(*bytes))
                break;
        }
        return true;
    }

    // The first block with `tag`, e.g. findBlock(file, fourCC("PHYS")).
    [[nodiscard]] inline std::optional<ByteView> findBlock(ByteView file, uint32_t tag) {
        std::optional<ByteView> found;
        forEachBlock(file, tag, [&](ByteView b) {
            found = b;
            return false;
        });
        return found;
    }

} // namespace s2ghtline
