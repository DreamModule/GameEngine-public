#include "Titan_Engine.hpp"
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdlib>

#if defined(_WIN32)
    #define WIN32_LEAN_AND_MEAN
    #include <windows.h>
#else
    #include <sys/mman.h>
    #include <unistd.h>
    #include <time.h>
    #include <pthread.h>
#endif

namespace Titan {

    f32 Math::Trig::Sin(f32 rad) { return std::sin(rad); }
    f32 Math::Trig::Cos(f32 rad) { return std::cos(rad); }
    f32 Math::Trig::Tan(f32 rad) { return std::tan(rad); }
    f32 Math::Trig::Sqrt(f32 val) { return std::sqrt(val); }
    f32 Math::Trig::InvSqrt(f32 val) { return 1.0f / std::sqrt(val); }
    f32 Math::Trig::RadToDeg(f32 rad) { return rad * (180.0f / PI); }
    f32 Math::Trig::DegToRad(f32 deg) { return deg * (PI / 180.0f); }

    Math::Vec3 Math::Vec3::Ops::Add(const Vec3& a, const Vec3& b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
    Math::Vec3 Math::Vec3::Ops::Sub(const Vec3& a, const Vec3& b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
    Math::Vec3 Math::Vec3::Ops::Mul(const Vec3& a, f32 s) { return {a.x*s, a.y*s, a.z*s}; }
    Math::Vec3 Math::Vec3::Ops::MulV(const Vec3& a, const Vec3& b) { return {a.x*b.x, a.y*b.y, a.z*b.z}; }
    f32  Math::Vec3::Ops::Dot(const Vec3& a, const Vec3& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
    
    Math::Vec3 Math::Vec3::Ops::Cross(const Vec3& a, const Vec3& b) {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        };
    }
    
    f32 Math::Vec3::Ops::Len(const Vec3& v) { return Trig::Sqrt(Dot(v, v)); }
    f32 Math::Vec3::Ops::LenSq(const Vec3& v) { return Dot(v, v); }
    
    Math::Vec3 Math::Vec3::Ops::Norm(const Vec3& v) {
        f32 l = Len(v);
        if (l > EPSILON) return Mul(v, 1.0f / l);
        return {0,0,0};
    }
    
    Math::Vec3 Math::Vec3::Ops::Lerp(const Vec3& a, const Vec3& b, f32 t) {
        return Add(a, Mul(Sub(b, a), t));
    }

    Math::Vec3 Math::Vec3::Ops::Reflect(const Vec3& dir, const Vec3& normal) {
        f32 d = Dot(dir, normal);
        return Sub(dir, Mul(normal, 2.0f * d));
    }

    Math::Mat4 Math::Mat4::Create::Identity() {
        Mat4 m;
        Memory::OS::Set(m.m, 0, sizeof(m));
        m.m[0][0] = 1; m.m[1][1] = 1; m.m[2][2] = 1; m.m[3][3] = 1;
        return m;
    }

    Math::Mat4 Math::Mat4::Create::Translate(const Vec3& t) {
        Mat4 m = Identity();
        m.m[3][0] = t.x; m.m[3][1] = t.y; m.m[3][2] = t.z;
        return m;
    }

    Math::Mat4 Math::Mat4::Create::Scale(const Vec3& s) {
        Mat4 m = Identity();
        m.m[0][0] = s.x; m.m[1][1] = s.y; m.m[2][2] = s.z;
        return m;
    }

    Math::Mat4 Math::Mat4::Create::Perspective(f32 fov, f32 aspect, f32 nearZ, f32 farZ) {
        Mat4 m = Zero();
        f32 tanHalf = Trig::Tan(fov * 0.5f);
        m.m[0][0] = 1.0f / (aspect * tanHalf);
        m.m[1][1] = 1.0f / tanHalf;
        m.m[2][2] = -(farZ + nearZ) / (farZ - nearZ);
        m.m[2][3] = -1.0f;
        m.m[3][2] = -(2.0f * farZ * nearZ) / (farZ - nearZ);
        return m;
    }

    Math::Mat4 Math::Mat4::Create::Zero() {
        Mat4 m; Memory::OS::Set(m.m, 0, sizeof(m)); return m;
    }

    Math::Mat4 Math::Mat4::Ops::Mul(const Mat4& a, const Mat4& b) {
        Mat4 r = Create::Zero();
        for(int c=0; c<4; ++c) {
            for(int rIdx=0; rIdx<4; ++rIdx) {
                for(int k=0; k<4; ++k) {
                    r.m[c][rIdx] += a.m[k][rIdx] * b.m[c][k];
                }
            }
        }
        return r;
    }

    bool Math::Geometry::Check::IntersectAABB(const AABB& a, const AABB& b) {
        return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
               (a.min.y <= b.max.y && a.max.y >= b.min.y) &&
               (a.min.z <= b.max.z && a.max.z >= b.min.z);
    }

    bool Math::Geometry::Check::IntersectSphere(const Sphere& a, const Sphere& b) {
        f32 distSq = Vec3::Ops::LenSq(Vec3::Ops::Sub(a.center, b.center));
        f32 radSum = a.radius + b.radius;
        return distSq <= (radSum * radSum);
    }

    void* Memory::OS::Reserve(usize size) {
#ifdef _WIN32
        return VirtualAlloc(0, size, MEM_RESERVE, PAGE_NOACCESS);
#else
        return mmap(nullptr, size, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
#endif
    }

    void Memory::OS::Commit(void* ptr, usize size) {
#ifdef _WIN32
        VirtualAlloc(ptr, size, MEM_COMMIT, PAGE_READWRITE);
#else
        mprotect(ptr, size, PROT_READ | PROT_WRITE);
#endif
    }

    void Memory::OS::Release(void* ptr) {
#ifdef _WIN32
        VirtualFree(ptr, 0, MEM_RELEASE);
#else
#endif
    }

    void Memory::OS::Copy(void* dst, const void* src, usize size) {
        std::memcpy(dst, src, size);
    }

    void Memory::OS::Set(void* dst, int val, usize size) {
        std::memset(dst, val, size);
    }

    void* Memory::Heap::Alloc(usize size) {
        return std::malloc(size);
    }

    void Memory::Heap::Free(void* ptr) {
        std::free(ptr);
    }

    void Memory::LinearArena::Control::Init(LinearArena& arena, usize size) {
        arena.start = (u8*)OS::Reserve(size);
        OS::Commit(arena.start, size);
        arena.current = arena.start;
        arena.end = arena.start + size;
    }

    void* Memory::LinearArena::Control::Alloc(LinearArena& arena, usize size, usize align) {
        ptr raw = (ptr)arena.current;
        ptr mask = align - 1;
        ptr aligned = (raw + mask) & ~mask;
        u8* next = (u8*)aligned + size;
        
        if (next > arena.end) return nullptr;
        
        arena.current = next;
        return (void*)aligned;
    }

    void Memory::LinearArena::Control::Reset(LinearArena& arena) {
        arena.current = arena.start;
    }

    u32 Data::Hash::FNV1a_32(const char* str) {
        u32 hash = 2166136261u;
        while (*str) {
            hash ^= (u8)*str++;
            hash *= 16777619u;
        }
        return hash;
    }

    Data::String Data::String::Ops::Create(const char* cstr, Memory::LinearArena* arena) {
        Data::String s;
        s.len = (u32)std::strlen(cstr);
        s.ptr = (char*)Memory::LinearArena::Control::Alloc(*arena, s.len + 1, 1);
        Memory::OS::Copy(s.ptr, cstr, s.len);
        s.ptr[s.len] = 0;
        return s;
    }

    bool Data::String::Ops::Equals(const String& a, const String& b) {
        if (a.len != b.len) return false;
        return std::memcmp(a.ptr, b.ptr, a.len) == 0;
    }

    void Data::RawArray::Ops::Init(RawArray& arr, u32 elemSize, u32 cap) {
        arr.elementSize = elemSize;
        arr.count = 0;
        arr.capacity = cap;
        arr.data = Memory::Heap::Alloc(cap * elemSize); 
    }

    void* Data::RawArray::Ops::Push(RawArray& arr) {
        if (arr.count >= arr.capacity) {
            return nullptr; 
        }
        void* ptr = (u8*)arr.data + (arr.count * arr.elementSize);
        arr.count++;
        return ptr;
    }

    static const char* g_DebugMenuTitle = "Debug";
    static bool g_DebugInitialized = false;

    void Debug::Menu::Init() {
        g_DebugInitialized = true;
    }

    void Debug::Menu::Create(const char* name) {
        if (!g_DebugInitialized) Init();
        g_DebugMenuTitle = name;
    }

    void Debug::Menu::Shutdown() {
        g_DebugInitialized = false;
    }

    void Debug::Menu::BeginFrame() {
        if (!g_DebugInitialized) return;
    }

    void Debug::Menu::EndFrame() {
        if (!g_DebugInitialized) return;
    }

    static f64 g_StartTime = 0;
    
    void Engine::Time::Init() {
#ifdef _WIN32
        LARGE_INTEGER li;
        QueryPerformanceCounter(&li);
        g_StartTime = (f64)li.QuadPart;
#else
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        g_StartTime = ts.tv_sec + ts.tv_nsec / 1e9;
#endif
    }

    f64 Engine::Time::GetAbsoluteTime() {
#ifdef _WIN32
        LARGE_INTEGER li, freq;
        QueryPerformanceCounter(&li);
        QueryPerformanceFrequency(&freq);
        return (f64)li.QuadPart / (f64)freq.QuadPart;
#else
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return ts.tv_sec + ts.tv_nsec / 1e9;
#endif
    }

    static Data::RawArray g_Components[Titan::Core::Limits::MAX_COMPONENTS];
    static u32 g_EntityCounter = 0;

    void Engine::ECS::Registry::Init() {
        for(int i=0; i<Titan::Core::Limits::MAX_COMPONENTS; ++i) {
            Data::RawArray::Ops::Init(g_Components[i], 0, 1024);
        }
    }

    Engine::ECS::EntityID Engine::ECS::Registry::CreateEntity() {
        return ++g_EntityCounter;
    }

    void* Engine::ECS::Registry::AddComponent(EntityID id, u32 compId, u32 compSize) {
        if (g_Components[compId].elementSize == 0) {
        }
        return nullptr; 
    }
}
