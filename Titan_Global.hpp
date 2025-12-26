#ifndef TITAN_GLOBAL_HPP
#define TITAN_GLOBAL_HPP

#include <cstdint>
#include <cstddef>
#include <atomic>
#include <new>

#if defined(_MSC_VER)
    #define TITAN_WIN 1
    #define TITAN_INLINE __forceinline
    #define TITAN_NOINLINE __declspec(noinline)
    #define TITAN_ALIGN(x) __declspec(align(x))
#else
    #define TITAN_LINUX 1
    #define TITAN_INLINE __attribute__((always_inline)) inline
    #define TITAN_NOINLINE __attribute__((noinline))
    #define TITAN_ALIGN(x) __attribute__((aligned(x)))
#endif

#define TITAN_KB(x) ((size_t)(x) << 10)
#define TITAN_MB(x) ((size_t)(x) << 20)
#define TITAN_GB(x) ((size_t)(x) << 30)

#define TITAN_ASSERT(x) do { if(!(x)) { *(volatile int*)0 = 0; } } while(0)

namespace Titan {
    using u8  = uint8_t;  using u16 = uint16_t;
    using u32 = uint32_t; using u64 = uint64_t;
    using i8  = int8_t;   using i16 = int16_t;
    using i32 = int32_t;  using i64 = int64_t;
    using f32 = float;    using f64 = double;
    using usize = size_t; using ptr = uintptr_t;

    struct Core {
        struct Limits {
            static constexpr u32 MAX_ENTITIES = 100000;
            static constexpr u32 MAX_COMPONENTS = 64;
            static constexpr usize PAGE_SIZE = 4096;
            static constexpr usize CACHE_LINE = 64;
        };
        
        struct Utils {
            template<typename T> static TITAN_INLINE T Min(T a, T b) { return a < b ? a : b; }
            template<typename T> static TITAN_INLINE T Max(T a, T b) { return a > b ? a : b; }
            template<typename T> static TITAN_INLINE T Clamp(T v, T min, T max) { 
                return (v < min) ? min : (v > max) ? max : v; 
            }
        };
    };
}

#endif
