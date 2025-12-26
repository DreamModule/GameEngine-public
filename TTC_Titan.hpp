#ifndef TTC_TITAN_HPP
#define TTC_TITAN_HPP

#include <cstdint>
#include <atomic>
#include <type_traits>
#include <immintrin.h>
#include <new>
#include <cstring>
#include <thread>

#if defined(_WIN32)
    #define TTC_WIN 1
    #define WIN32_LEAN_AND_MEAN
    #include <windows.h>
#elif defined(__linux__) || defined(__APPLE__)
    #define TTC_LINUX 1
    #include <sys/mman.h>
    #include <unistd.h>
#endif

#define TTC_FORCE_INLINE __attribute__((always_inline)) inline
#define TTC_ALIGN(x) alignas(x)
#define TTC_KB(x) ((size_t)(x) << 10)
#define TTC_MB(x) ((size_t)(x) << 20)
#define TTC_CACHE_LINE 64

namespace TTC {
    using u8 = uint8_t; using u16 = uint16_t; using u32 = uint32_t; using u64 = uint64_t;
    using i8 = int8_t; using i16 = int16_t; using i32 = int32_t; using i64 = int64_t;
    using f32 = float; using f64 = double; using usize = size_t; using ptr = uintptr_t;

    struct NonCopyable {
        NonCopyable() = default;
        NonCopyable(const NonCopyable&) = delete;
        NonCopyable& operator=(const NonCopyable&) = delete;
    };
}

namespace TTC::OS {
    inline usize GetPageSize() { return 4096; }
    
    inline void* VirtualReserve(usize size) {
#if TTC_WIN
        return VirtualAlloc(0, size, MEM_RESERVE, PAGE_NOACCESS);
#else
        return mmap(nullptr, size, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
#endif
    }

    inline void VirtualCommit(void* ptr, usize size) {
#if TTC_WIN
        VirtualAlloc(ptr, size, MEM_COMMIT, PAGE_READWRITE);
#else
        mprotect(ptr, size, PROT_READ | PROT_WRITE);
#endif
    }

    inline void VirtualFree(void* ptr, usize size) {
#if TTC_WIN
        VirtualFree(ptr, 0, MEM_RELEASE);
#else
        munmap(ptr, size);
#endif
    }
}

namespace TTC::Memory {
    struct ScratchArena : NonCopyable {
        u8* base; u8* end; u8* current; u8* committed;
        
        void Init(usize maxSize) {
            base = (u8*)OS::VirtualReserve(maxSize);
            end = base + maxSize;
            current = base;
            committed = base;
        }

        void* Alloc(usize size, usize align = 8) {
            ptr raw = (ptr)current;
            ptr mask = align - 1;
            ptr alignedPtr = (raw + mask) & ~mask;
            u8* next = (u8*)alignedPtr + size;
            if (next > end) return nullptr;
            if (next > committed) {
                usize needed = (next - committed + 4095) & ~4095;
                OS::VirtualCommit(committed, needed);
                committed += needed;
            }
            current = next;
            return (void*)alignedPtr;
        }

        void Reset() { current = base; }
    };
    extern thread_local ScratchArena g_TLSArena;
}

namespace TTC::Container {
    template<typename T>
    struct Array {
        T* data; u32 size; u32 capacity;
        
        void Init(u32 cap = 16) {
            size = 0; capacity = cap;
            usize bytes = sizeof(T) * cap;
            data = (T*)OS::VirtualReserve(bytes);
            OS::VirtualCommit(data, bytes);
        }

        void Push(const T& val) {
            if (size >= capacity) {
                u32 newCap = capacity * 2;
                T* newData = (T*)OS::VirtualReserve(sizeof(T) * newCap);
                OS::VirtualCommit(newData, sizeof(T) * newCap);
                memcpy(newData, data, sizeof(T) * size);
                OS::VirtualFree(data, sizeof(T) * capacity);
                data = newData;
                capacity = newCap;
            }
            data[size++] = val;
        }
        T& operator[](u32 i) { return data[i]; }
        const T& operator[](u32 i) const { return data[i]; }
    };
}

namespace TTC::Math {
    struct F32x8 {
        __m256 v;
        F32x8() : v(_mm256_setzero_ps()) {}
        F32x8(__m256 _v) : v(_v) {}
        static F32x8 Load(const f32* p) { return _mm256_loadu_ps(p); }
        void Store(f32* p) const { _mm256_storeu_ps(p, v); }
        F32x8 operator+(const F32x8& b) const { return _mm256_add_ps(v, b.v); }
        F32x8 operator*(const F32x8& b) const { return _mm256_mul_ps(v, b.v); }
        static F32x8 FMA(const F32x8& a, const F32x8& b, const F32x8& c) { return _mm256_fmadd_ps(a.v, b.v, c.v); }
    };
}

namespace TTC::Job {
    struct Context { void* rip; void* rsp; void* rbx; void* rbp; void* r12; void* r13; void* r14; void* r15; };
    struct Job;
    struct Fiber { Context ctx; void* stackBase; usize stackSize; Job* currentJob; };
    
    struct Job {
        void (*function)(void*, void*);
        void* data;
        std::atomic<i32>* counter;
    };

    template<typename T, u32 C>
    struct MPMCQueue {
        struct Slot { std::atomic<u32> seq; T data; };
        Slot buf[C];
        TTC_ALIGN(64) std::atomic<u32> head;
        TTC_ALIGN(64) std::atomic<u32> tail;

        MPMCQueue() {
            for(u32 i=0; i<C; ++i) buf[i].seq.store(i, std::memory_order_relaxed);
            head.store(0); tail.store(0);
        }

        bool Enqueue(const T& d) {
            Slot* s; u32 pos = head.load(std::memory_order_relaxed);
            while(true) {
                s = &buf[pos & (C-1)];
                u32 seq = s->seq.load(std::memory_order_acquire);
                i32 diff = (i32)seq - (i32)pos;
                if(diff == 0) {
                    if(head.compare_exchange_weak(pos, pos+1, std::memory_order_relaxed)) break;
                } else if(diff < 0) return false;
                else pos = head.load(std::memory_order_relaxed);
            }
            s->data = d;
            s->seq.store(pos+1, std::memory_order_release);
            return true;
        }

        bool Dequeue(T& d) {
            Slot* s; u32 pos = tail.load(std::memory_order_relaxed);
            while(true) {
                s = &buf[pos & (C-1)];
                u32 seq = s->seq.load(std::memory_order_acquire);
                i32 diff = (i32)seq - (i32)(pos+1);
                if(diff == 0) {
                    if(tail.compare_exchange_weak(pos, pos+1, std::memory_order_relaxed)) break;
                } else if(diff < 0) return false;
                else pos = tail.load(std::memory_order_relaxed);
            }
            d = s->data;
            s->seq.store(pos+C, std::memory_order_release);
            return true;
        }
    };

    extern "C" void TTC_SwitchContext(void** oldSp, void* newSp);

    struct Scheduler {
        static constexpr u32 MAX_THREADS = 16;
        MPMCQueue<Job, 1024> queues[MAX_THREADS];
        std::atomic<bool> running{true};
        
        void Schedule(Job j, u32 tid) { queues[tid].Enqueue(j); }
        
        void WorkerLoop(u32 tid) {
            while(running.load(std::memory_order_relaxed)) {
                Job j;
                if(queues[tid].Dequeue(j)) {
                    j.function(j.data, nullptr);
                    if(j.counter) j.counter->fetch_sub(1, std::memory_order_release);
                    continue;
                }
                for(u32 i=1; i<MAX_THREADS; ++i) {
                    if(queues[(tid+i)%MAX_THREADS].Dequeue(j)) {
                        j.function(j.data, nullptr);
                        if(j.counter) j.counter->fetch_sub(1, std::memory_order_release);
                        break;
                    }
                }
                _mm_pause();
            }
        }
    };
}

namespace TTC::ECS {
    static constexpr usize CHUNK_SIZE = 16 * 1024;
    static constexpr u32 MAX_COMPS = 32;

    struct ComponentType { u16 id; u16 size; };
    struct ChunkHeader { u32 archId; u16 count; u16 cap; u16 offsets[MAX_COMPS]; };
    struct Chunk { ChunkHeader header; u8 data[CHUNK_SIZE - sizeof(ChunkHeader)]; };
    
    struct Archetype {
        u32 id; u32 compCount;
        ComponentType types[MAX_COMPS];
        Container::Array<Chunk*> chunks;
    };

    struct Record { Archetype* arch; Chunk* chunk; u16 index; };

    struct World {
        Container::Array<Archetype*> archetypes;
        Container::Array<Record> entityIndex;

        void Init() {
            archetypes.Init(64);
            entityIndex.Init(100000);
        }

        u8* GetComponent(Chunk* c, u16 offset, u16 idx, u16 size) {
            return c->data + offset + (idx * size);
        }
    };
}

#endif
