cmake_minimum_required(VERSION 3.21)
project(s2ghtline VERSION 1.0.0 DESCRIPTION "Source 2 map collision: VPK / KV3 / PHYS reader, BVH, line of sight" LANGUAGES C CXX)

option(S2GHTLINE_BUILD_TESTS    "Build the tests"                                     ${PROJECT_IS_TOP_LEVEL})
option(S2GHTLINE_BUILD_EXAMPLES "Build the example"                                   ${PROJECT_IS_TOP_LEVEL})
option(S2GHTLINE_SANITIZE       "Build with AddressSanitizer (+ UBSan on GCC / Clang)" OFF)
option(S2GHTLINE_WERROR         "Treat warnings as errors in the project's own code"  OFF)

find_package(Threads REQUIRED)

if(S2GHTLINE_SANITIZE)
    if(MSVC)
        add_compile_options(/fsanitize=address /Zi)
        add_link_options(/INCREMENTAL:NO /DEBUG)
    else()
        add_compile_options(-fsanitize=address,undefined -fno-sanitize-recover=undefined -fno-omit-frame-pointer -g)
        add_link_options(-fsanitize=address,undefined)
    endif()
endif()

# ── Third-party decompressors (C): lz4 1.10.0, the zstd 1.5.7 decoder ─────
add_library(s2ghtline_third_party STATIC third_party/lz4/lz4.c third_party/zstd/zstddeclib.c)
add_library(s2ghtline::third_party ALIAS s2ghtline_third_party)
target_include_directories(s2ghtline_third_party PUBLIC "${PROJECT_SOURCE_DIR}/third_party")   # <lz4/lz4.h>, <zstd/zstd.h>
set_target_properties(s2ghtline_third_party PROPERTIES POSITION_INDEPENDENT_CODE ON C_STANDARD 11)
target_compile_options(s2ghtline_third_party PRIVATE $<IF:$<C_COMPILER_ID:MSVC>,/W0,-w>)

# ── The library (header-only) ─────────────────────────────────────────────
# Headers include each other by bare name ("Kv3.h"); include/s2ghtline is on the
# path for that, include/ too so users can write <s2ghtline/World.h>.
add_library(s2ghtline INTERFACE)
add_library(s2ghtline::s2ghtline ALIAS s2ghtline)
target_include_directories(s2ghtline INTERFACE "${PROJECT_SOURCE_DIR}/include/s2ghtline" "${PROJECT_SOURCE_DIR}/include")
target_compile_features(s2ghtline INTERFACE cxx_std_20)
target_link_libraries(s2ghtline INTERFACE s2ghtline_third_party Threads::Threads)
if(WIN32)
    target_compile_definitions(s2ghtline INTERFACE NOMINMAX)
endif()

# Warnings for the project's own code.
function(s2ghtline_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive- /utf-8 $<$<BOOL:${S2GHTLINE_WERROR}>:/WX>)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion
                               $<$<BOOL:${S2GHTLINE_WERROR}>:-Werror>)
    endif()
endfunction()

# ── Example ───────────────────────────────────────────────────────────────
if(S2GHTLINE_BUILD_EXAMPLES)
    add_executable(raycast examples/raycast.cpp)
    target_link_libraries(raycast PRIVATE s2ghtline)
    target_include_directories(raycast PRIVATE "${PROJECT_SOURCE_DIR}/tests")   # support/Cs2Install.h
    s2ghtline_warnings(raycast)
endif()

# ── Tests ─────────────────────────────────────────────────────────────────
# Label "synthetic": generated input only, run anywhere (CI).
# Label "cs2": read the installed game; exit 77 (skipped) when none is found.
if(S2GHTLINE_BUILD_TESTS)
    enable_testing()

    function(s2ghtline_test name label timeout)
        add_executable(${name} tests/${name}.cpp)
        target_link_libraries(${name} PRIVATE s2ghtline)
        target_include_directories(${name} PRIVATE "${PROJECT_SOURCE_DIR}/tests")
        s2ghtline_warnings(${name})
        add_test(NAME ${name} COMMAND ${name})
        set_tests_properties(${name} PROPERTIES LABELS ${label} TIMEOUT ${timeout} SKIP_RETURN_CODE 77)
    endfunction()

    s2ghtline_test(test_bvh      synthetic 300)
    s2ghtline_test(test_kv3      synthetic 600)
    s2ghtline_test(test_world    synthetic 300)
    s2ghtline_test(test_surfaces synthetic 60)

    s2ghtline_test(test_bvh_maps      cs2 1200)
    s2ghtline_test(test_kv3_maps      cs2 1200)
    s2ghtline_test(test_world_maps    cs2 600)
    s2ghtline_test(test_contacts      cs2 1200)
    s2ghtline_test(test_surfaces_maps cs2 120)

    # The C++ side of the VRF comparison (tests/compare_oracle.py); not a CTest test.
    add_executable(map_check tests/map_check.cpp)
    target_link_libraries(map_check PRIVATE s2ghtline)
    s2ghtline_warnings(map_check)
endif()
