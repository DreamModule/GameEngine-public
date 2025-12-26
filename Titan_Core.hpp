#ifndef TITAN_CORE_HPP
#define TITAN_CORE_HPP

#include <cstdint>
#include <cstddef>
#include <cfloat>
#include <cmath>
#include <cstring>
#include <new>
#include <atomic>
#include <immintrin.h>

#if defined(_MSC_VER)
    #define TTC_INLINE __forceinline
    #define TTC_NOINLINE __declspec(noinline)
    #define TTC_ALIGN(x) __declspec(align(x))
#else
    #define TTC_INLINE __attribute__((always_inline)) inline
    #define TTC_NOINLINE __attribute__((noinline))
    #define TTC_ALIGN(x) __attribute__((aligned(x)))
#endif

#define TTC_ASSERT(x) do { if(!(x)) { *(volatile int*)0 = 0; } } while(0)
#define TTC_KB(x) ((size_t)(x) << 10)
#define TTC_MB(x) ((size_t)(x) << 20)
#define TTC_GB(x) ((size_t)(x) << 30)

namespace TTC {
    using u8  = uint8_t;  using u16 = uint16_t;
    using u32 = uint32_t; using u64 = uint64_t;
    using i8  = int8_t;   using i16 = int16_t;
    using i32 = int32_t;  using i64 = int64_t;
    using f32 = float;    using f64 = double;
    using usize = size_t; using ptr = uintptr_t;

    template<typename T> struct RemoveReference { using Type = T; };
    template<typename T> struct RemoveReference<T&> { using Type = T; };
    template<typename T> struct RemoveReference<T&&> { using Type = T; };
    
    template<typename T>
    TTC_INLINE T&& Move(T& obj) { return (T&&)obj; }
    
    template<typename T>
    TTC_INLINE T&& Forward(typename RemoveReference<T>::Type& obj) { return (T&&)obj; }
    
    template<typename T>
    TTC_INLINE T&& Forward(typename RemoveReference<T>::Type&& obj) { return (T&&)obj; }

    template<typename T> struct IsPOD { static const bool Value = false; };
    template<> struct IsPOD<u8> { static const bool Value = true; };
    template<> struct IsPOD<u16> { static const bool Value = true; };
    template<> struct IsPOD<u32> { static const bool Value = true; };
    template<> struct IsPOD<u64> { static const bool Value = true; };
    template<> struct IsPOD<i8> { static const bool Value = true; };
    template<> struct IsPOD<i16> { static const bool Value = true; };
    template<> struct IsPOD<i32> { static const bool Value = true; };
    template<> struct IsPOD<i64> { static const bool Value = true; };
    template<> struct IsPOD<f32> { static const bool Value = true; };
    template<> struct IsPOD<f64> { static const bool Value = true; };

    template<typename T>
    TTC_INLINE void Swap(T& a, T& b) {
        T tmp = Move(a);
        a = Move(b);
        b = Move(tmp);
    }

    template<typename T>
    TTC_INLINE T Min(T a, T b) { return a < b ? a : b; }
    
    template<typename T>
    TTC_INLINE T Max(T a, T b) { return a > b ? a : b; }

    template<typename T>
    TTC_INLINE T Clamp(T v, T lo, T hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }
    struct IAllocator {
        virtual void* Alloc(usize size, usize align) = 0;
        virtual void Free(void* ptr) = 0;
        virtual ~IAllocator() {}
    };

    struct MallocAllocator : IAllocator {
        void* Alloc(usize size, usize align) override {
            void* ptr = nullptr;
#if defined(_MSC_VER)
            ptr = _aligned_malloc(size, align);
#else
            posix_memalign(&ptr, align, size);
#endif
            return ptr;
        }
        void Free(void* ptr) override {
#if defined(_MSC_VER)
            _aligned_free(ptr);
#else
            free(ptr);
#endif
        }
    };
    static MallocAllocator g_HeapAllocator;
    namespace Math {
        static constexpr f32 PI = 3.1415926535897932f;
        static constexpr f32 EPSILON = 1.192092896e-07F;
        
        TTC_INLINE f32 Sqrt(f32 x) { return _mm_cvtss_f32(_mm_sqrt_ss(_mm_set_ss(x))); }
        TTC_INLINE f32 RSqrt(f32 x) { return _mm_cvtss_f32(_mm_rsqrt_ss(_mm_set_ss(x))); }
        TTC_INLINE f32 Sin(f32 x) { return sinf(x); } // Intrinsics preferred in prod
        TTC_INLINE f32 Cos(f32 x) { return cosf(x); }
        TTC_INLINE f32 Tan(f32 x) { return tanf(x); }
        TTC_INLINE f32 Deg2Rad(f32 deg) { return deg * (PI / 180.0f); }
        TTC_INLINE f32 Rad2Deg(f32 rad) { return rad * (180.0f / PI); }
        struct Vec2 {
            f32 x, y;
            Vec2() : x(0), y(0) {}
            Vec2(f32 _x, f32 _y) : x(_x), y(_y) {}
            
            bool operator==(const Vec2& r) const { return x==r.x && y==r.y; }
            bool operator!=(const Vec2& r) const { return !(*this == r); }
            Vec2 operator+(const Vec2& r) const { return {x+r.x, y+r.y}; }
            Vec2 operator-(const Vec2& r) const { return {x-r.x, y-r.y}; }
            Vec2 operator*(f32 s) const { return {x*s, y*s}; }
            Vec2 operator/(f32 s) const { return {x/s, y/s}; }
            void operator+=(const Vec2& r) { x+=r.x; y+=r.y; }
            void operator-=(const Vec2& r) { x-=r.x; y-=r.y; }
            
            f32 Dot(const Vec2& r) const { return x*r.x + y*r.y; }
            f32 LenSq() const { return x*x + y*y; }
            f32 Len() const { return Sqrt(LenSq()); }
            Vec2 Norm() const { f32 l = Len(); return l > EPSILON ? *this * (1.0f/l) : Vec2(0,0); }
        };
        struct TTC_ALIGN(16) Vec3 {
            union {
                struct { f32 x, y, z; };
                __m128 mm;
            };

            Vec3() : mm(_mm_setzero_ps()) {}
            Vec3(f32 _x, f32 _y, f32 _z) : mm(_mm_set_ps(0, _z, _y, _x)) {}
            Vec3(__m128 m) : mm(m) {}

            Vec3 operator+(const Vec3& r) const { return Vec3(_mm_add_ps(mm, r.mm)); }
            Vec3 operator-(const Vec3& r) const { return Vec3(_mm_sub_ps(mm, r.mm)); }
            Vec3 operator*(f32 s) const { return Vec3(_mm_mul_ps(mm, _mm_set1_ps(s))); }
            Vec3 operator*(const Vec3& r) const { return Vec3(_mm_mul_ps(mm, r.mm)); }
            
            f32 Dot(const Vec3& r) const {
                __m128 m = _mm_mul_ps(mm, r.mm);
                m = _mm_add_ps(m, _mm_movehl_ps(m, m));
                m = _mm_add_ss(m, _mm_shuffle_ps(m, m, 1));
                return _mm_cvtss_f32(m);
            }

            Vec3 Cross(const Vec3& b) const {
                __m128 a_yzx = _mm_shuffle_ps(mm, mm, _MM_SHUFFLE(3, 0, 2, 1));
                __m128 b_yzx = _mm_shuffle_ps(b.mm, b.mm, _MM_SHUFFLE(3, 0, 2, 1));
                __m128 a_zxy = _mm_shuffle_ps(mm, mm, _MM_SHUFFLE(3, 1, 0, 2));
                __m128 b_zxy = _mm_shuffle_ps(b.mm, b.mm, _MM_SHUFFLE(3, 1, 0, 2));
                return Vec3(_mm_sub_ps(_mm_mul_ps(a_yzx, b_zxy), _mm_mul_ps(a_zxy, b_yzx)));
            }

            f32 Len() const { return Sqrt(Dot(*this)); }
            Vec3 Norm() const { 
                f32 l = Len(); 
                return l > EPSILON ? *this * (1.0f/l) : Vec3(0,0,0); 
            }
        };
        struct TTC_ALIGN(16) Vec4 {
            union {
                struct { f32 x, y, z, w; };
                __m128 mm;
            };
            
            Vec4() : mm(_mm_setzero_ps()) {}
            Vec4(f32 _x, f32 _y, f32 _z, f32 _w) : mm(_mm_set_ps(_w, _z, _y, _x)) {}
            Vec4(__m128 m) : mm(m) {}
            
            Vec4 operator+(const Vec4& r) const { return Vec4(_mm_add_ps(mm, r.mm)); }
            Vec4 operator-(const Vec4& r) const { return Vec4(_mm_sub_ps(mm, r.mm)); }
            Vec4 operator*(f32 s) const { return Vec4(_mm_mul_ps(mm, _mm_set1_ps(s))); }
        };
        struct Quat {
            f32 x, y, z, w;
            
            Quat() : x(0), y(0), z(0), w(1) {}
            Quat(f32 _x, f32 _y, f32 _z, f32 _w) : x(_x), y(_y), z(_z), w(_w) {}
            
            static Quat Identity() { return Quat(0,0,0,1); }
            
            static Quat FromAxisAngle(const Vec3& axis, f32 angle) {
                f32 half = angle * 0.5f;
                f32 s = Sin(half);
                return Quat(axis.x * s, axis.y * s, axis.z * s, Cos(half));
            }

            Quat operator*(const Quat& r) const {
                return Quat(
                    w*r.x + x*r.w + y*r.z - z*r.y,
                    w*r.y - x*r.z + y*r.w + z*r.x,
                    w*r.z + x*r.y - y*r.x + z*r.w,
                    w*r.w - x*r.x - y*r.y - z*r.z
                );
            }

            Vec3 Rotate(const Vec3& v) const {
                Vec3 u(x, y, z);
                f32 s = w;
                return u * 2.0f * u.Dot(v)
                     + v * (s*s - u.Dot(u))
                     + u.Cross(v) * 2.0f * s;
            }
        };
        struct TTC_ALIGN(16) Mat4 {
            __m128 cols[4];

            static Mat4 Identity() {
                Mat4 m;
                m.cols[0] = _mm_setr_ps(1,0,0,0);
                m.cols[1] = _mm_setr_ps(0,1,0,0);
                m.cols[2] = _mm_setr_ps(0,0,1,0);
                m.cols[3] = _mm_setr_ps(0,0,0,1);
                return m;
            }

            static Mat4 Translate(const Vec3& t) {
                Mat4 m = Identity();
                m.cols[3] = _mm_setr_ps(t.x, t.y, t.z, 1);
                return m;
            }

            static Mat4 Scale(const Vec3& s) {
                Mat4 m;
                m.cols[0] = _mm_setr_ps(s.x,0,0,0);
                m.cols[1] = _mm_setr_ps(0,s.y,0,0);
                m.cols[2] = _mm_setr_ps(0,0,s.z,0);
                m.cols[3] = _mm_setr_ps(0,0,0,1);
                return m;
            }

            static Mat4 Perspective(f32 fov, f32 aspect, f32 nearZ, f32 farZ) {
                Mat4 m;
                f32 tanHalf = Tan(fov * 0.5f);
                m.cols[0] = _mm_setr_ps(1.0f / (aspect * tanHalf), 0, 0, 0);
                m.cols[1] = _mm_setr_ps(0, 1.0f / tanHalf, 0, 0);
                m.cols[2] = _mm_setr_ps(0, 0, -(farZ + nearZ) / (farZ - nearZ), -1.0f);
                m.cols[3] = _mm_setr_ps(0, 0, -(2.0f * farZ * nearZ) / (farZ - nearZ), 0);
                return m;
            }

            static Mat4 LookAt(const Vec3& eye, const Vec3& center, const Vec3& up) {
                Vec3 f = (center - eye).Norm();
                Vec3 s = f.Cross(up).Norm();
                Vec3 u = s.Cross(f);

                Mat4 m;
                m.cols[0] = _mm_setr_ps(s.x, u.x, -f.x, 0);
                m.cols[1] = _mm_setr_ps(s.y, u.y, -f.y, 0);
                m.cols[2] = _mm_setr_ps(s.z, u.z, -f.z, 0);
                m.cols[3] = _mm_setr_ps(-s.Dot(eye), -u.Dot(eye), f.Dot(eye), 1);
                return m;
            }

            Mat4 operator*(const Mat4& b) const {
                Mat4 res;
                for (int i = 0; i < 4; i++) {
                    res.cols[i] = _mm_add_ps(
                        _mm_add_ps(
                            _mm_mul_ps(_mm_shuffle_ps(b.cols[i], b.cols[i], 0x00), cols[0]),
                            _mm_mul_ps(_mm_shuffle_ps(b.cols[i], b.cols[i], 0x55), cols[1])),
                        _mm_add_ps(
                            _mm_mul_ps(_mm_shuffle_ps(b.cols[i], b.cols[i], 0xAA), cols[2]),
                            _mm_mul_ps(_mm_shuffle_ps(b.cols[i], b.cols[i], 0xFF), cols[3]))
                    );
                }
                return res;
            }
        };
        struct AABB {
            Vec3 min, max;
            bool Intersects(const AABB& o) const {
                return (min.x <= o.max.x && max.x >= o.min.x) &&
                       (min.y <= o.max.y && max.y >= o.min.y) &&
                       (min.z <= o.max.z && max.z >= o.min.z);
            }
            void Expand(const Vec3& p) {
                min.x = Min(min.x, p.x); min.y = Min(min.y, p.y); min.z = Min(min.z, p.z);
                max.x = Max(max.x, p.x); max.y = Max(max.y, p.y); max.z = Max(max.z, p.z);
            }
        };

        struct Plane {
            Vec3 n; f32 d;
            f32 Dist(const Vec3& p) const { return n.Dot(p) + d; }
        };

        struct Frustum {
            Plane planes[6];
            bool Contains(const AABB& box) const {
                for(int i=0; i<6; ++i) {
                    Vec3 p = box.min;
                    if (planes[i].n.x >= 0) p.x = box.max.x;
                    if (planes[i].n.y >= 0) p.y = box.max.y;
                    if (planes[i].n.z >= 0) p.z = box.max.z;
                    if (planes[i].Dist(p) < 0) return false;
                }
                return true;
            }
        };
    }
    namespace Container {
        struct String {
            char* data;
            u32 len;
            u32 cap;
            IAllocator* allocator;

            String(IAllocator* a = &g_HeapAllocator) : data(nullptr), len(0), cap(0), allocator(a) {}
            String(const char* str, IAllocator* a = &g_HeapAllocator) : allocator(a) {
                len = (u32)strlen(str);
                cap = len + 1;
                data = (char*)allocator->Alloc(cap, 1);
                memcpy(data, str, len);
                data[len] = 0;
            }
            ~String() { if(data) allocator->Free(data); }

            String(const String& o) : allocator(o.allocator) {
                len = o.len; cap = o.cap;
                if(cap > 0) {
                    data = (char*)allocator->Alloc(cap, 1);
                    memcpy(data, o.data, len + 1);
                } else data = nullptr;
            }

            void Append(const char* str) {
                u32 slen = (u32)strlen(str);
                if (len + slen >= cap) {
                    u32 newCap = (cap == 0) ? (slen + 16) : (cap * 2 + slen);
                    char* newData = (char*)allocator->Alloc(newCap, 1);
                    if(data) {
                        memcpy(newData, data, len);
                        allocator->Free(data);
                    }
                    data = newData;
                    cap = newCap;
                }
                memcpy(data + len, str, slen);
                len += slen;
                data[len] = 0;
            }
            
            bool operator==(const char* other) const { return strcmp(data, other) == 0; }
            const char* CStr() const { return data ? data : ""; }
        };

        template<typename T>
        struct Vector {
            T* data;
            u32 size;
            u32 capacity;
            IAllocator* allocator;

            Vector(IAllocator* a = &g_HeapAllocator) : data(nullptr), size(0), capacity(0), allocator(a) {}
            ~Vector() { Release(); }

            void Release() {
                if(data) {
                    if(!IsPOD<T>::Value) {
                        for(u32 i=0; i<size; ++i) data[i].~T();
                    }
                    allocator->Free(data);
                }
                data = nullptr; size = 0; capacity = 0;
            }

            void Reserve(u32 newCap) {
                if (newCap <= capacity) return;
                T* newData = (T*)allocator->Alloc(newCap * sizeof(T), alignof(T));
                if (data) {
                    for(u32 i=0; i<size; ++i) {
                        new (&newData[i]) T(Move(data[i]));
                        if(!IsPOD<T>::Value) data[i].~T();
                    }
                    allocator->Free(data);
                }
                data = newData;
                capacity = newCap;
            }

            void Push(const T& val) {
                if (size == capacity) Reserve(capacity ? capacity * 2 : 16);
                new (&data[size++]) T(val);
            }
            
            void Push(T&& val) {
                if (size == capacity) Reserve(capacity ? capacity * 2 : 16);
                new (&data[size++]) T(Move(val));
            }

            void Pop() {
                if(size > 0) {
                    size--;
                    if(!IsPOD<T>::Value) data[size].~T();
                }
            }

            T& operator[](u32 i) { TTC_ASSERT(i < size); return data[i]; }
            const T& operator[](u32 i) const { TTC_ASSERT(i < size); return data[i]; }
            
            T* begin() { return data; }
            T* end() { return data + size; }
        };

        template<typename K, typename V>
        struct HashMap {
            struct Entry { K key; V val; bool used; };
            Entry* entries;
            u32 capacity;
            u32 size;
            IAllocator* allocator;

            HashMap(IAllocator* a = &g_HeapAllocator) : entries(nullptr), capacity(0), size(0), allocator(a) {}
            ~HashMap() { if(entries) allocator->Free(entries); }

            u32 Hash(u32 x) { 
                x = ((x >> 16) ^ x) * 0x45d9f3b;
                x = ((x >> 16) ^ x) * 0x45d9f3b;
                return (x >> 16) ^ x;
            }

            void Reserve(u32 newCap) {
                Entry* oldEntries = entries;
                u32 oldCap = capacity;
                
                capacity = newCap;
                entries = (Entry*)allocator->Alloc(sizeof(Entry) * capacity, alignof(Entry));
                memset(entries, 0, sizeof(Entry) * capacity);
                size = 0;

                if(oldEntries) {
                    for(u32 i=0; i<oldCap; ++i) {
                        if(oldEntries[i].used) Insert(oldEntries[i].key, oldEntries[i].val);
                    }
                    allocator->Free(oldEntries);
                }
            }

            void Insert(const K& key, const V& val) {
                if(size * 2 >= capacity) Reserve(capacity ? capacity * 2 : 32);
                
                u32 h = Hash(*(u32*)&key); // Hacky hash for POD
                u32 idx = h & (capacity - 1);
                
                while(entries[idx].used) {
                    if(entries[idx].key == key) { entries[idx].val = val; return; }
                    idx = (idx + 1) & (capacity - 1);
                }
                
                entries[idx].key = key;
                entries[idx].val = val;
                entries[idx].used = true;
                size++;
            }
            
            V* Get(const K& key) {
                if(capacity == 0) return nullptr;
                u32 h = Hash(*(u32*)&key);
                u32 idx = h & (capacity - 1);
                while(entries[idx].used) {
                    if(entries[idx].key == key) return &entries[idx].val;
                    idx = (idx + 1) & (capacity - 1);
                }
                return nullptr;
            }
        };
    }
}

#endif 
