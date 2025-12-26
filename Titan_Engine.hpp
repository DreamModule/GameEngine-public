#ifndef TITAN_ENGINE_HPP
#define TITAN_ENGINE_HPP

#include "Titan_Global.hpp"

namespace Titan {

    struct Math {
        static constexpr f32 PI = 3.14159265359f;
        static constexpr f32 EPSILON = 0.000001f;

        struct Trig {
            static f32 Sin(f32 rad);
            static f32 Cos(f32 rad);
            static f32 Tan(f32 rad);
            static f32 Sqrt(f32 val);
            static f32 InvSqrt(f32 val);
            static f32 RadToDeg(f32 rad);
            static f32 DegToRad(f32 deg);
        };

        struct Vec3 {
            f32 x, y, z;
            
            struct Ops {
                static Vec3 Add(const Vec3& a, const Vec3& b);
                static Vec3 Sub(const Vec3& a, const Vec3& b);
                static Vec3 Mul(const Vec3& a, f32 s);
                static Vec3 MulV(const Vec3& a, const Vec3& b);
                static f32  Dot(const Vec3& a, const Vec3& b);
                static Vec3 Cross(const Vec3& a, const Vec3& b);
                static f32  Len(const Vec3& v);
                static f32  LenSq(const Vec3& v);
                static Vec3 Norm(const Vec3& v);
                static Vec3 Lerp(const Vec3& a, const Vec3& b, f32 t);
                static Vec3 Reflect(const Vec3& dir, const Vec3& normal);
            };
        };

        struct Mat4 {
            f32 m[4][4];

            struct Create {
                static Mat4 Identity();
                static Mat4 Zero();
                static Mat4 Translate(const Vec3& t);
                static Mat4 Scale(const Vec3& s);
                static Mat4 RotateX(f32 rad);
                static Mat4 RotateY(f32 rad);
                static Mat4 RotateZ(f32 rad);
                static Mat4 Perspective(f32 fov, f32 aspect, f32 nearZ, f32 farZ);
                static Mat4 Orthographic(f32 left, f32 right, f32 bot, f32 top, f32 nearZ, f32 farZ);
                static Mat4 LookAt(const Vec3& eye, const Vec3& target, const Vec3& up);
            };

            struct Ops {
                static Mat4 Mul(const Mat4& a, const Mat4& b);
                static Vec3 MulPoint(const Mat4& m, const Vec3& p);
                static Vec3 MulDir(const Mat4& m, const Vec3& v);
                static Mat4 Inverse(const Mat4& m);
                static Mat4 Transpose(const Mat4& m);
            };
        };
        
        struct Geometry {
            struct AABB { Vec3 min; Vec3 max; };
            struct Sphere { Vec3 center; f32 radius; };
            struct Ray { Vec3 origin; Vec3 dir; };
            struct Plane { Vec3 n; f32 d; };

            struct Check {
                static bool IntersectAABB(const AABB& a, const AABB& b);
                static bool IntersectSphere(const Sphere& a, const Sphere& b);
                static bool PointInAABB(const Vec3& p, const AABB& b);
                static bool RayPlane(const Ray& r, const Plane& p, f32& t);
            };
        };
    };

    struct Memory {
        struct OS {
            static void* Reserve(usize size);
            static void  Commit(void* ptr, usize size);
            static void  Decommit(void* ptr, usize size);
            static void  Release(void* ptr);
            static void  Copy(void* dst, const void* src, usize size);
            static void  Set(void* dst, int val, usize size);
        };

        struct Allocator {
            struct IAlloc {
                void* (*Alloc)(void* self, usize size, usize align);
                void  (*Free)(void* self, void* ptr);
            };
        };

        struct LinearArena {
            u8* start;
            u8* current;
            u8* end;
            
            struct Control {
                static void Init(LinearArena& arena, usize size);
                static void* Alloc(LinearArena& arena, usize size, usize align);
                static void Reset(LinearArena& arena);
                static void Destroy(LinearArena& arena);
            };
        };

        struct Heap {
            static void* Alloc(usize size);
            static void  Free(void* ptr);
        };
    };

    struct Data {
        struct Hash {
            static u32 FNV1a_32(const char* str);
            static u64 FNV1a_64(const char* str);
            static u32 CRC32(const void* data, usize size);
        };

        struct String {
            char* ptr;
            u32 len;
            
            struct Ops {
                static String Create(const char* cstr, Memory::LinearArena* arena);
                static bool   Equals(const String& a, const String& b);
                static void   Copy(String& dst, const char* src);
                static String Concat(const String& a, const String& b, Memory::LinearArena* arena);
                static i32    ToInt(const String& s);
                static f32    ToFloat(const String& s);
            };
            
            struct Path {
                static String GetExtension(const String& path, Memory::LinearArena* arena);
                static String GetFileName(const String& path, Memory::LinearArena* arena);
                static String GetDirectory(const String& path, Memory::LinearArena* arena);
            };
        };

        struct RawArray {
            void* data;
            u32 count;
            u32 capacity;
            u32 elementSize;

            struct Ops {
                static void Init(RawArray& arr, u32 elemSize, u32 cap);
                static void* Push(RawArray& arr);
                static void* Get(RawArray& arr, u32 index);
                static void Clear(RawArray& arr);
            };
        };
    };

    struct Debug {
        struct Menu {
            static void Init();
            static void Create(const char* name = "Debug");
            static void Shutdown();
            static void BeginFrame();
            static void EndFrame();
        };
    };

    struct Engine {
        
        struct Time {
            static void Init();
            static f64  GetAbsoluteTime();
            static f32  GetDeltaTime();
            static void Sleep(u32 ms);
        };

        struct Threading {
            struct Job {
                void (*Func)(void*);
                void* Data;
            };

            struct Scheduler {
                static void Init(u32 threadCount);
                static void PushJob(Job job);
                static void WaitIdle();
                static void Shutdown();
            };
            
            struct Atomics {
                static i32 Increment(volatile i32* val);
                static i32 Decrement(volatile i32* val);
                static bool CompareExchange(volatile i32* val, i32 exchange, i32 comparand);
            };
        };

        struct ECS {
            typedef u32 EntityID;
            
            struct Component {
                u32 id;
                u32 size;
            };

            struct Registry {
                static void Init();
                static EntityID CreateEntity();
                static void DestroyEntity(EntityID id);
                static void* AddComponent(EntityID id, u32 compId, u32 compSize);
                static void* GetComponent(EntityID id, u32 compId);
                static bool HasComponent(EntityID id, u32 compId);
            };
        };
    };
}

#endif
