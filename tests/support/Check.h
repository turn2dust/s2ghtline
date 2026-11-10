#pragma once

// Check.h — the tests' tiny harness: named checks, a failure count, and the
// exit codes CTest reads (0 passed, 1 failed, 77 skipped).

#include <cstdio>
#include <string>

namespace check {

    inline constexpr int kSkipped = 77;   // CTest SKIP_RETURN_CODE

    inline int& failures() {
        static int n = 0;
        return n;
    }

    inline void section(const char* name) { std::printf("== %s\n", name); }

    inline bool expect(bool ok, const char* what) {
        std::printf("  %s %s\n", ok ? "ok  " : "FAIL", what);
        if (!ok)
            ++failures();
        return ok;
    }

    // main()'s return value.
    inline int result() {
        std::printf(failures() ? "FAILED (%d)\n" : "all passed\n", failures());
        return failures() ? 1 : 0;
    }

    inline int skip(const char* why) {
        std::printf("SKIPPED: %s\n", why);
        return kSkipped;
    }

    // A real-map test without a game: skipped - unless one was named
    // (S2GHTLINE_CS2_DIR) and isn't there, which fails.
    inline int noGame(const std::string& why, bool named) {
        if (!named)
            return skip(why.c_str());
        std::printf("FAIL %s\n", why.c_str());
        return 1;
    }

} // namespace check
