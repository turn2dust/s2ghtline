# Third-party notices

s2ghtline includes the following third-party code, unmodified, under
`third_party/`. Each component's full license text is in the LICENSE file
next to it.

| Component | Version | Files | License |
|---|---|---|---|
| [LZ4](https://github.com/lz4/lz4) | 1.10.0 | `third_party/lz4/lz4.c`, `lz4.h` | BSD 2-Clause ([`third_party/lz4/LICENSE`](third_party/lz4/LICENSE)) |
| [Zstandard](https://github.com/facebook/zstd), decoder only | 1.5.7 | `third_party/zstd/zstddeclib.c` (the single-file decoder from `build/single_file_libs`), `zstd.h`, `zstd_errors.h` | BSD 3-Clause ([`third_party/zstd/LICENSE`](third_party/zstd/LICENSE)); upstream is dual-licensed BSD / GPLv2, and s2ghtline uses it under the BSD license |

## LZ4

Copyright (c) 2011-2020, Yann Collet. All rights reserved.
Licensed under the BSD 2-Clause License; see `third_party/lz4/LICENSE`.

## Zstandard

Copyright (c) Meta Platforms, Inc. and affiliates. All rights reserved.
Licensed under the BSD License; see `third_party/zstd/LICENSE`.

## Development tools (not part of the library)

`tools/MapOracle`, the reference used to check the parser, downloads
[ValveResourceFormat](https://github.com/ValveResourceFormat/ValveResourceFormat)
20.0.6980 (MIT) from NuGet when it is built. Nothing from it ships with or
links into s2ghtline.
