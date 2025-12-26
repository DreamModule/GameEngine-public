#ifndef TTC_ENGINE_MONOLITH_HPP
#define TTC_ENGINE_MONOLITH_HPP

#include <cstdint>
#include <cassert>
#include <cstring>
#include <type_traits>
#include <new>
#include <utility>
#include <limits>
#include <cstdlib>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <algorithm>
#include <functional>
#include <tuple>
#include <chrono>
#include <cmath>

#define TTC_LOG(Level, Message) std::cout << "[TTC][" << #Level << "] " << Message << std::endl
#define TTC_ERROR(Message) { std::cerr << "[FATAL] " << Message << std::endl; std::terminate(); }

namespace TTC {
namespace Memory {

    static constexpr size_t ALIGNMENT = 16;

    inline void* AlignForward(void* p, size_t align) {
        uintptr_t pi = reinterpret_cast<uintptr_t>(p);
        const size_t mod = pi % align;
        if (mod == 0) return p;
        return reinterpret_cast<void*>(pi + align - mod);
    }

    class IAllocator {
    public:
        virtual ~IAllocator() = default;
        virtual void* Allocate(size_t size, size_t align = ALIGNMENT) = 0;
        virtual void Deallocate(void* p) = 0;
        virtual void Reset() = 0;
        virtual size_t GetUsedMemory() const = 0;
        virtual size_t GetTotalMemory() const = 0;
    };

    class HeapAllocator : public IAllocator {
    private:
        size_t totalAllocated = 0;
    public:
        void* Allocate(size_t size, size_t align = ALIGNMENT) override {
            size_t totalSize = size + align + sizeof(size_t);
            uint8_t* raw = static_cast<uint8_t*>(std::malloc(totalSize));
            if (!raw) return nullptr;

            uint8_t* aligned = static_cast<uint8_t*>(AlignForward(raw + sizeof(size_t), align));
            size_t* header = reinterpret_cast<size_t*>(aligned - sizeof(size_t));
            *header = static_cast<size_t>(aligned - raw);
            
            totalAllocated += totalSize;
            return aligned;
        }

        void Deallocate(void* p) override {
            if (!p) return;
            uint8_t* aligned = static_cast<uint8_t*>(p);
            size_t* header = reinterpret_cast<size_t*>(aligned - sizeof(size_t));
            size_t offset = *header;
            uint8_t* raw = aligned - offset;
            std::free(raw);
        }

        void Reset() override { }
        size_t GetUsedMemory() const override { return totalAllocated; }
        size_t GetTotalMemory() const override { return std::numeric_limits<size_t>::max(); }
    };

    class LinearAllocator : public IAllocator {
    private:
        void* startPtr;
        size_t totalSize;
        size_t offset;
    public:
        LinearAllocator(size_t size) : totalSize(size), offset(0) {
            startPtr = std::malloc(size);
        }
        ~LinearAllocator() { std::free(startPtr); }

        void* Allocate(size_t size, size_t align = ALIGNMENT) override {
            uintptr_t currentAddress = reinterpret_cast<uintptr_t>(startPtr) + offset;
            uintptr_t padding = 0;
            uintptr_t mod = currentAddress % align;
            if (mod != 0) padding = align - mod;

            if (offset + padding + size > totalSize) {
                return nullptr;
            }

            offset += padding;
            void* nextAddress = reinterpret_cast<void*>(reinterpret_cast<uintptr_t>(startPtr) + offset);
            offset += size;
            return nextAddress;
        }

        void Deallocate(void* p) override { }
        void Reset() override { offset = 0; }
        size_t GetUsedMemory() const override { return offset; }
        size_t GetTotalMemory() const override { return totalSize; }
    };

    extern IAllocator* GlobalAllocator; 
}
}

namespace TTC {
namespace Containers {

    template <typename T>
    class Vector {
    private:
        T* data;
        size_t capacity;
        size_t length;
        TTC::Memory::IAllocator* allocator;

        void Realloc(size_t newCapacity) {
            T* newData = static_cast<T*>(allocator->Allocate(newCapacity * sizeof(T)));
            if (newCapacity < length) length = newCapacity;

            for (size_t i = 0; i < length; i++) {
                new (&newData[i]) T(std::move(data[i]));
                data[i].~T();
            }
            if (data) allocator->Deallocate(data);
            data = newData;
            capacity = newCapacity;
        }

    public:
        Vector(TTC::Memory::IAllocator* alloc = nullptr) 
            : data(nullptr), capacity(0), length(0) {
            if (alloc) allocator = alloc;
            else allocator = new TTC::Memory::HeapAllocator();
        }

        ~Vector() {
            Clear();
            if (data) allocator->Deallocate(data);
        }

        void PushBack(const T& value) {
            if (length >= capacity) {
                Realloc(capacity == 0 ? 8 : capacity * 2);
            }
            new (&data[length]) T(value);
            length++;
        }

        T& operator[](size_t index) {
            assert(index < length && "Vector index out of bounds");
            return data[index];
        }
        
        const T& operator[](size_t index) const {
            assert(index < length && "Vector index out of bounds");
            return data[index];
        }

        size_t Size() const { return length; }
        
        void Clear() {
            for (size_t i = 0; i < length; i++) {
                data[i].~T();
            }
            length = 0;
        }

        T* Begin() { return data; }
        T* End() { return data + length; }
    };
}
}

namespace TTC {
namespace Utils {

    using StringHash = uint32_t;

    constexpr StringHash HashFNV1a(const char* str, size_t n, StringHash basis = 2166136261U) {
        return n == 0 ? basis : HashFNV1a(str + 1, n - 1, (basis ^ str[0]) * 16777619U);
    }

    template <size_t N>
    constexpr StringHash ConstHash(const char (&str)[N]) {
        return HashFNV1a(str, N - 1);
    }

    class StringId {
    private:
        StringHash value;
    public:
        constexpr StringId() : value(0) {}
        constexpr StringId(StringHash hash) : value(hash) {}
        
        StringId(const char* str) {
            value = HashFNV1a(str, std::strlen(str));
        }

        constexpr bool operator==(const StringId& other) const { return value == other.value; }
        constexpr bool operator!=(const StringId& other) const { return value != other.value; }
        constexpr bool operator<(const StringId& other) const { return value < other.value; }
        
        StringHash GetHash() const { return value; }
    };
}
}

namespace TTC {
namespace Math {

    struct Fixed32 {
        int32_t raw;
        static constexpr int32_t SHIFT = 16;
        constexpr Fixed32() : raw(0) {}
        constexpr Fixed32(int32_t v) : raw(v) {}
        static Fixed32 FromInt(int v) { return Fixed32(v << SHIFT); }
        static Fixed32 FromFloat(float v) { return Fixed32((int32_t)(v * 65536.0f)); }
        
        Fixed32 operator+(const Fixed32& b) const { return Fixed32(raw + b.raw); }
        Fixed32 operator-(const Fixed32& b) const { return Fixed32(raw - b.raw); }
        Fixed32 operator*(const Fixed32& b) const { 
            return Fixed32((int32_t)(((int64_t)raw * b.raw) >> SHIFT)); 
        }
        Fixed32 operator/(const Fixed32& b) const { 
             if(b.raw == 0) return Fixed32(0);
             return Fixed32((int32_t)(((int64_t)raw << SHIFT) / b.raw)); 
        }
        
        bool operator<(const Fixed32& rhs) const { return raw < rhs.raw; }
        bool operator>(const Fixed32& rhs) const { return raw > rhs.raw; }
        bool operator<=(const Fixed32& rhs) const { return raw <= rhs.raw; }
        bool operator>=(const Fixed32& rhs) const { return raw >= rhs.raw; }

        static Fixed32 Zero() { return Fixed32(0); }
        static Fixed32 One() { return Fixed32(1 << SHIFT); }
    };

    struct Vector2 {
        Fixed32 x, y;
        constexpr Vector2() : x(0), y(0) {}
        constexpr Vector2(Fixed32 _x, Fixed32 _y) : x(_x), y(_y) {}
        
        static Vector2 FromInts(int _x, int _y) {
            return Vector2(Fixed32::FromInt(_x), Fixed32::FromInt(_y));
        }

        Vector2 operator+(const Vector2& rhs) const { return Vector2(x + rhs.x, y + rhs.y); }
        Vector2 operator-(const Vector2& rhs) const { return Vector2(x - rhs.x, y - rhs.y); }
        Vector2 operator*(const Fixed32& scalar) const { return Vector2(x * scalar, y * scalar); }
    };

    struct Vector3 {
        Fixed32 x, y, z;
        Vector3() : x(0), y(0), z(0) {}
        Vector3(Fixed32 _x, Fixed32 _y, Fixed32 _z) : x(_x), y(_y), z(_z) {}

        Vector3 operator+(const Vector3& b) const { return Vector3(x + b.x, y + b.y, z + b.z); }
        Vector3 operator-(const Vector3& b) const { return Vector3(x - b.x, y - b.y, z - b.z); }
        Vector3 operator*(Fixed32 s) const { return Vector3(x * s, y * s, z * s); }
        
        Fixed32 Dot(const Vector3& b) const { return x*b.x + y*b.y + z*b.z; }
        
        Vector3 Cross(const Vector3& b) const {
            return Vector3(
                y * b.z - z * b.y,
                z * b.x - x * b.z,
                x * b.y - y * b.x
            );
        }
    };

    struct Matrix4 {
        Fixed32 m[4][4];

        Matrix4() { Identity(); }

        void Identity() {
            m[0][0] = Fixed32::One(); m[0][1] = Fixed32::Zero(); m[0][2] = Fixed32::Zero(); m[0][3] = Fixed32::Zero();
            m[1][0] = Fixed32::Zero(); m[1][1] = Fixed32::One(); m[1][2] = Fixed32::Zero(); m[1][3] = Fixed32::Zero();
            m[2][0] = Fixed32::Zero(); m[2][1] = Fixed32::Zero(); m[2][2] = Fixed32::One(); m[2][3] = Fixed32::Zero();
            m[3][0] = Fixed32::Zero(); m[3][1] = Fixed32::Zero(); m[3][2] = Fixed32::Zero(); m[3][3] = Fixed32::One();
        }

        Matrix4 operator*(const Matrix4& rhs) const {
            Matrix4 r;
            r.m[0][0] = m[0][0]*rhs.m[0][0] + m[0][1]*rhs.m[1][0] + m[0][2]*rhs.m[2][0] + m[0][3]*rhs.m[3][0];
            r.m[0][1] = m[0][0]*rhs.m[0][1] + m[0][1]*rhs.m[1][1] + m[0][2]*rhs.m[2][1] + m[0][3]*rhs.m[3][1];
            r.m[0][2] = m[0][0]*rhs.m[0][2] + m[0][1]*rhs.m[1][2] + m[0][2]*rhs.m[2][2] + m[0][3]*rhs.m[3][2];
            r.m[0][3] = m[0][0]*rhs.m[0][3] + m[0][1]*rhs.m[1][3] + m[0][2]*rhs.m[2][3] + m[0][3]*rhs.m[3][3];

            r.m[1][0] = m[1][0]*rhs.m[0][0] + m[1][1]*rhs.m[1][0] + m[1][2]*rhs.m[2][0] + m[1][3]*rhs.m[3][0];
            r.m[1][1] = m[1][0]*rhs.m[0][1] + m[1][1]*rhs.m[1][1] + m[1][2]*rhs.m[2][1] + m[1][3]*rhs.m[3][1];
            r.m[1][2] = m[1][0]*rhs.m[0][2] + m[1][1]*rhs.m[1][2] + m[1][2]*rhs.m[2][2] + m[1][3]*rhs.m[3][2];
            r.m[1][3] = m[1][0]*rhs.m[0][3] + m[1][1]*rhs.m[1][3] + m[1][2]*rhs.m[2][3] + m[1][3]*rhs.m[3][3];

            r.m[2][0] = m[2][0]*rhs.m[0][0] + m[2][1]*rhs.m[1][0] + m[2][2]*rhs.m[2][0] + m[2][3]*rhs.m[3][0];
            r.m[2][1] = m[2][0]*rhs.m[0][1] + m[2][1]*rhs.m[1][1] + m[2][2]*rhs.m[2][1] + m[2][3]*rhs.m[3][1];
            r.m[2][2] = m[2][0]*rhs.m[0][2] + m[2][1]*rhs.m[1][2] + m[2][2]*rhs.m[2][2] + m[2][3]*rhs.m[3][2];
            r.m[2][3] = m[2][0]*rhs.m[0][3] + m[2][1]*rhs.m[1][3] + m[2][2]*rhs.m[2][3] + m[2][3]*rhs.m[3][3];

            r.m[3][0] = m[3][0]*rhs.m[0][0] + m[3][1]*rhs.m[1][0] + m[3][2]*rhs.m[2][0] + m[3][3]*rhs.m[3][0];
            r.m[3][1] = m[3][0]*rhs.m[0][1] + m[3][1]*rhs.m[1][1] + m[3][2]*rhs.m[2][1] + m[3][3]*rhs.m[3][1];
            r.m[3][2] = m[3][0]*rhs.m[0][2] + m[3][1]*rhs.m[1][2] + m[3][2]*rhs.m[2][2] + m[3][3]*rhs.m[3][2];
            r.m[3][3] = m[3][0]*rhs.m[0][3] + m[3][1]*rhs.m[1][3] + m[3][2]*rhs.m[2][3] + m[3][3]*rhs.m[3][3];
            return r;
        }

        static Matrix4 Translate(const Vector3& v) {
            Matrix4 r;
            r.m[0][3] = v.x;
            r.m[1][3] = v.y;
            r.m[2][3] = v.z;
            return r;
        }

        static Matrix4 Scale(const Vector3& v) {
            Matrix4 r;
            r.m[0][0] = v.x;
            r.m[1][1] = v.y;
            r.m[2][2] = v.z;
            return r;
        }
        
        Matrix4 Transpose() const {
            Matrix4 r;
            for(int i=0; i<4; i++)
                for(int j=0; j<4; j++)
                    r.m[i][j] = m[j][i];
            return r;
        }
    };
}
}

namespace TTC {
namespace Input {

    enum class Key : uint16_t {
        None = 0,
        A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
        Num0, Num1, Num2, Num3, Num4, Num5, Num6, Num7, Num8, Num9,
        Escape, Enter, Space, Backspace, Tab, Shift, Control, Alt,
        ArrowUp, ArrowDown, ArrowLeft, ArrowRight,
        F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12,
        MouseLeft, MouseRight, MouseMiddle,
        Count
    };

    enum class KeyState : uint8_t {
        Up = 0,
        Down = 1,
        JustPressed = 2,
        JustReleased = 3
    };

    class InputManager {
    private:
        KeyState keyStates[static_cast<int>(Key::Count)];
        bool currentFrameState[static_cast<int>(Key::Count)];
        bool previousFrameState[static_cast<int>(Key::Count)];
        
        TTC::Math::Vector2 mousePosition;
        TTC::Math::Vector2 mouseDelta;
        TTC::Math::Fixed32 scrollDelta;

    public:
        InputManager() {
            std::memset(keyStates, 0, sizeof(keyStates));
            std::memset(currentFrameState, 0, sizeof(currentFrameState));
            std::memset(previousFrameState, 0, sizeof(previousFrameState));
        }

        void Update() {
            for (int i = 0; i < static_cast<int>(Key::Count); ++i) {
                bool isDown = currentFrameState[i];
                bool wasDown = previousFrameState[i];

                if (isDown && !wasDown) keyStates[i] = KeyState::JustPressed;
                else if (!isDown && wasDown) keyStates[i] = KeyState::JustReleased;
                else if (isDown) keyStates[i] = KeyState::Down;
                else keyStates[i] = KeyState::Up;

                previousFrameState[i] = currentFrameState[i];
            }
            mouseDelta = TTC::Math::Vector2(0, 0);
            scrollDelta = TTC::Math::Fixed32::Zero();
        }

        void ProcessEvent(Key key, bool pressed) {
            currentFrameState[static_cast<int>(key)] = pressed;
        }

        void ProcessMouse(float x, float y) {
            TTC::Math::Vector2 newPos = TTC::Math::Vector2::FromInts((int)x, (int)y); 
            mouseDelta = newPos - mousePosition;
            mousePosition = newPos;
        }

        bool GetKey(Key key) const {
            return currentFrameState[static_cast<int>(key)];
        }

        bool GetKeyDown(Key key) const {
            return keyStates[static_cast<int>(key)] == KeyState::JustPressed;
        }

        bool GetKeyUp(Key key) const {
            return keyStates[static_cast<int>(key)] == KeyState::JustReleased;
        }

        TTC::Math::Vector2 GetMousePosition() const { return mousePosition; }
        TTC::Math::Vector2 GetMouseDelta() const { return mouseDelta; }
    };

    extern InputManager* GInput;
}
}

namespace TTC {
namespace Time {
    class Clock {
    private:
        std::chrono::high_resolution_clock::time_point startTime;
        std::chrono::high_resolution_clock::time_point lastFrameTime;
        double deltaTime;
        double timeScale;

    public:
        Clock() : deltaTime(0.016), timeScale(1.0) {
            startTime = std::chrono::high_resolution_clock::now();
            lastFrameTime = startTime;
        }

        void Update() {
            auto current = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> diff = current - lastFrameTime;
            deltaTime = diff.count();
            lastFrameTime = current;
        }

        double GetDeltaTime() const { return deltaTime * timeScale; }
        TTC::Math::Fixed32 GetFixedDeltaTime() const { 
            return TTC::Math::Fixed32::FromFloat((float)(deltaTime * timeScale)); 
        }
        void SetTimeScale(double scale) { timeScale = scale; }
    };
    extern Clock* GClock;
}
}

namespace TTC {
namespace Debug {
    
    using namespace TTC::Math;

    enum class DrawCommandType {
        WindowBegin,
        WindowEnd,
        Text,
        Button,
        Checkbox,
        SliderFixed,
        PlotLines,
        Separator,
        SameLine
    };

    struct DrawCommand {
        DrawCommandType type;
        const char* label;
        union {
            struct { bool* bVal; } checkbox;
            struct { Fixed32* fVal; Fixed32 min; Fixed32 max; } slider;
            struct { const Fixed32* values; int count; } plot;
            struct { Fixed32 x; Fixed32 y; } size;
        } data;
        bool resultClicked; 
    };

    class DebugLayer {
    private:
        TTC::Containers::Vector<DrawCommand> commandBuffer;
        bool isFrameOpen;
        
        bool CheckClick(const char* id) {
            return false; 
        }

    public:
        DebugLayer() : isFrameOpen(false) {}

        void BeginFrame() {
            commandBuffer.Clear();
            isFrameOpen = true;
        }

        void EndFrame() {
            isFrameOpen = false;
        }

        void Draw(std::function<void()> drawCallback) {
            if (!isFrameOpen) BeginFrame();
            drawCallback();
            if (isFrameOpen) EndFrame();
        }

        bool BeginWindow(const char* title, const Vector2& pos, const Vector2& size) {
            DrawCommand cmd;
            cmd.type = DrawCommandType::WindowBegin;
            cmd.label = title;
            cmd.data.size.x = pos.x;
            cmd.data.size.y = pos.y;
            commandBuffer.PushBack(cmd);
            return true;
        }

        void EndWindow() {
            DrawCommand cmd;
            cmd.type = DrawCommandType::WindowEnd;
            commandBuffer.PushBack(cmd);
        }

        void Text(const char* fmt, ...) {
            DrawCommand cmd;
            cmd.type = DrawCommandType::Text;
            cmd.label = fmt;
            commandBuffer.PushBack(cmd);
        }

        bool Button(const char* label, const Vector2& size = Vector2()) {
            DrawCommand cmd;
            cmd.type = DrawCommandType::Button;
            cmd.label = label;
            cmd.data.size = size;
            cmd.resultClicked = CheckClick(label);
            commandBuffer.PushBack(cmd);
            return cmd.resultClicked;
        }

        bool Checkbox(const char* label, bool* v) {
            DrawCommand cmd;
            cmd.type = DrawCommandType::Checkbox;
            cmd.label = label;
            cmd.data.checkbox.bVal = v;
            bool clicked = Button(label); 
            if (clicked) *v = !(*v);
            commandBuffer.PushBack(cmd);
            return clicked;
        }

        bool SliderFixed(const char* label, Fixed32* v, Fixed32 v_min, Fixed32 v_max) {
            DrawCommand cmd;
            cmd.type = DrawCommandType::SliderFixed;
            cmd.label = label;
            cmd.data.slider.fVal = v;
            cmd.data.slider.min = v_min;
            cmd.data.slider.max = v_max;
            commandBuffer.PushBack(cmd);
            return false; 
        }

        void PlotLines(const char* label, const Fixed32* values, int count) {
            DrawCommand cmd;
            cmd.type = DrawCommandType::PlotLines;
            cmd.label = label;
            cmd.data.plot.values = values;
            cmd.data.plot.count = count;
            commandBuffer.PushBack(cmd);
        }
        
        void Separator() {
            DrawCommand cmd;
            cmd.type = DrawCommandType::Separator;
            commandBuffer.PushBack(cmd);
        }
    };

    extern DebugLayer* GDebug;
}
}

namespace TTC {
namespace Core {

    using EntityID = uint32_t;
    const EntityID NULL_ENTITY = 0;

    class IComponentPool {
    public:
        virtual ~IComponentPool() = default;
        virtual void Remove(EntityID entity) = 0;
    };

    template<typename T>
    class Pool : public IComponentPool {
    private:
        std::vector<T> data;
        std::map<EntityID, size_t> entityToIndex;
        std::map<size_t, EntityID> indexToEntity;

    public:
        void Add(EntityID entity, T component) {
            if (entityToIndex.find(entity) != entityToIndex.end()) {
                data[entityToIndex[entity]] = component;
                return;
            }
            size_t index = data.size();
            data.push_back(component);
            entityToIndex[entity] = index;
            indexToEntity[index] = entity;
        }

        void Remove(EntityID entity) override {
            if (entityToIndex.find(entity) == entityToIndex.end()) return;
            size_t indexToRemove = entityToIndex[entity];
            size_t lastIndex = data.size() - 1;
            EntityID lastEntity = indexToEntity[lastIndex];
            data[indexToRemove] = data[lastIndex];
            entityToIndex[lastEntity] = indexToRemove;
            indexToEntity[indexToRemove] = lastEntity;
            entityToIndex.erase(entity);
            indexToEntity.erase(lastIndex);
            data.pop_back();
        }

        T& Get(EntityID entity) {
            if (entityToIndex.find(entity) == entityToIndex.end()) {
                TTC_ERROR("Component missing: " << entity);
            }
            return data[entityToIndex[entity]];
        }
        
        bool Has(EntityID entity) const {
            return entityToIndex.find(entity) != entityToIndex.end();
        }
    };

    class Registry {
    private:
        EntityID nextEntityId = 1;
        std::vector<EntityID> activeEntities;
        std::map<size_t, std::shared_ptr<IComponentPool>> pools;

        template<typename T>
        std::shared_ptr<Pool<T>> GetPool() {
            size_t typeHash = typeid(T).hash_code();
            if (pools.find(typeHash) == pools.end()) {
                pools[typeHash] = std::make_shared<Pool<T>>();
            }
            return std::static_pointer_cast<Pool<T>>(pools[typeHash]);
        }

    public:
        EntityID Create() {
            EntityID id = nextEntityId++;
            activeEntities.push_back(id);
            return id;
        }

        template<typename T>
        void Emplace(EntityID entity, T component) {
            GetPool<T>()->Add(entity, component);
        }

        template<typename T>
        T& Get(EntityID entity) {
            return GetPool<T>()->Get(entity);
        }
        
        template<typename T>
        bool Has(EntityID entity) {
            return GetPool<T>()->Has(entity);
        }
    };

    template<typename... Components>
    class View {
    private:
        Registry* registry;
        std::vector<EntityID> entities;

    public:
        View(Registry* reg) : registry(reg) {
            
        }

        struct Iterator {
            Registry* reg;
            std::vector<EntityID>::iterator current;

            Iterator(Registry* r, std::vector<EntityID>::iterator c) : reg(r), current(c) {}

            Iterator& operator++() {
                current++;
                return *this;
            }

            bool operator!=(const Iterator& other) const {
                return current != other.current;
            }

            std::tuple<EntityID, Components&...> operator*() {
                EntityID id = *current;
                return std::tuple<EntityID, Components&...>(id, reg->Get<Components>(id)...);
            }
        };

        Iterator begin() { return Iterator(registry, entities.begin()); }
        Iterator end() { return Iterator(registry, entities.end()); }
    };

    class EngineContext {
    private:
        TTC::Memory::IAllocator* mainAllocator;
        TTC::Memory::LinearAllocator* frameAllocator;
        bool isRunning;
        uint64_t tickCount;

    public:
        EngineContext() : isRunning(false), tickCount(0) {
            size_t mainHeapSize = 1024 * 1024 * 1024;
            mainAllocator = new TTC::Memory::HeapAllocator(); 
            frameAllocator = new TTC::Memory::LinearAllocator(64 * 1024 * 1024);
        }

        ~EngineContext() {
            delete frameAllocator;
            delete mainAllocator;
        }

        void Initialize() {
            isRunning = true;
        }

        void Update() {
            if (!isRunning) return;
            frameAllocator->Reset();
            tickCount++;
            if (TTC::Input::GInput) TTC::Input::GInput->Update();
            if (TTC::Time::GClock) TTC::Time::GClock->Update();
        }
        
        uint64_t GetTick() const { return tickCount; }
        TTC::Memory::IAllocator* GetMainAllocator() { return mainAllocator; }
        TTC::Memory::LinearAllocator* GetFrameAllocator() { return frameAllocator; }
    };

    extern EngineContext* GEngine;

}
}

namespace TTC {
namespace Math {

    struct Quaternion {
        Fixed32 x, y, z, w;

        Quaternion() : x(0), y(0), z(0), w(Fixed32::One()) {}
        Quaternion(Fixed32 _x, Fixed32 _y, Fixed32 _z, Fixed32 _w) : x(_x), y(_y), z(_z), w(_w) {}

        static Quaternion Identity() { return Quaternion(Fixed32::Zero(), Fixed32::Zero(), Fixed32::Zero(), Fixed32::One()); }

        Quaternion operator*(const Quaternion& b) const {
            return Quaternion(
                w * b.x + x * b.w + y * b.z - z * b.y,
                w * b.y + y * b.w + z * b.x - x * b.z,
                w * b.z + z * b.w + x * b.y - y * b.x,
                w * b.w - x * b.x - y * b.y - z * b.z
            );
        }

        Vector3 operator*(const Vector3& v) const {
            Vector3 u(x, y, z);
            Fixed32 s = w;
            Vector3 result = u * (u.Dot(v) * Fixed32::FromInt(2))
                + v * (s * s - u.Dot(u))
                + u.Cross(v) * (s * Fixed32::FromInt(2));
            return result;
        }

        Fixed32 Dot(const Quaternion& b) const {
            return x * b.x + y * b.y + z * b.z + w * b.w;
        }

        Quaternion Normalized() const {
            Fixed32 lenSq = Dot(*this);
            if (lenSq <= Fixed32::Epsilon()) return Identity();
            Fixed32 len = Fixed32::Sqrt(lenSq);
            return Quaternion(x / len, y / len, z / len, w / len);
        }

        static Quaternion Slerp(const Quaternion& a, const Quaternion& b, Fixed32 t) {
            Quaternion z = b;
            Fixed32 cosTheta = a.Dot(b);
            if (cosTheta < Fixed32::Zero()) {
                z = Quaternion(-b.x, -b.y, -b.z, -b.w);
                cosTheta = -cosTheta;
            }
            if (cosTheta > Fixed32::FromFloat(0.9995f)) {
                return Quaternion(
                    a.x + (z.x - a.x) * t,
                    a.y + (z.y - a.y) * t,
                    a.z + (z.z - a.z) * t,
                    a.w + (z.w - a.w) * t
                ).Normalized();
            }
            Fixed32 angle = Fixed32::FromFloat(std::acos(cosTheta.ToFloat()));
            Fixed32 s1 = Fixed32::FromFloat(std::sin((Fixed32::One() - t).ToFloat() * angle.ToFloat()));
            Fixed32 s2 = Fixed32::FromFloat(std::sin(t.ToFloat() * angle.ToFloat()));
            Fixed32 s3 = Fixed32::FromFloat(1.0f / std::sin(angle.ToFloat()));
            return Quaternion(
                (a.x * s1 + z.x * s2) * s3,
                (a.y * s1 + z.y * s2) * s3,
                (a.z * s1 + z.z * s2) * s3,
                (a.w * s1 + z.w * s2) * s3
            );
        }

        Matrix4 ToMatrix() const {
            Matrix4 m;
            Fixed32 xx = x * x; Fixed32 yy = y * y; Fixed32 zz = z * z;
            Fixed32 xy = x * y; Fixed32 xz = x * z; Fixed32 yz = y * z;
            Fixed32 wx = w * x; Fixed32 wy = w * y; Fixed32 wz = w * z;

            m.m[0][0] = Fixed32::One() - (yy + zz) * Fixed32::FromInt(2);
            m.m[0][1] = (xy - wz) * Fixed32::FromInt(2);
            m.m[0][2] = (xz + wy) * Fixed32::FromInt(2);
            m.m[0][3] = Fixed32::Zero();

            m.m[1][0] = (xy + wz) * Fixed32::FromInt(2);
            m.m[1][1] = Fixed32::One() - (xx + zz) * Fixed32::FromInt(2);
            m.m[1][2] = (yz - wx) * Fixed32::FromInt(2);
            m.m[1][3] = Fixed32::Zero();

            m.m[2][0] = (xz - wy) * Fixed32::FromInt(2);
            m.m[2][1] = (yz + wx) * Fixed32::FromInt(2);
            m.m[2][2] = Fixed32::One() - (xx + yy) * Fixed32::FromInt(2);
            m.m[2][3] = Fixed32::Zero();

            return m;
        }
    };
    
    struct Easing {
        static Fixed32 Linear(Fixed32 t) { return t; }
        static Fixed32 InQuad(Fixed32 t) { return t * t; }
        static Fixed32 OutQuad(Fixed32 t) { return t * (Fixed32::FromInt(2) - t); }
        static Fixed32 InOutQuad(Fixed32 t) {
            if (t < Fixed32::FromFloat(0.5f)) return t * t * Fixed32::FromInt(2);
            return (Fixed32::FromInt(4) - t * Fixed32::FromInt(2)) * t - Fixed32::One();
        }
    };
}
}

namespace TTC {
namespace IO {

    class BinaryWriter {
    private:
        std::vector<uint8_t> buffer;
    public:
        void WriteInt32(int32_t value) {
            size_t s = buffer.size();
            buffer.resize(s + 4);
            std::memcpy(buffer.data() + s, &value, 4);
        }
        void WriteFixed(TTC::Math::Fixed32 value) {
            WriteInt32(value.raw);
        }
        void WriteVector2(const TTC::Math::Vector2& v) {
            WriteFixed(v.x);
            WriteFixed(v.y);
        }
        void WriteString(const std::string& str) {
            WriteInt32((int32_t)str.size());
            size_t s = buffer.size();
            buffer.resize(s + str.size());
            std::memcpy(buffer.data() + s, str.data(), str.size());
        }
        const std::vector<uint8_t>& GetBuffer() const { return buffer; }
    };

    class BinaryReader {
    private:
        const uint8_t* data;
        size_t size;
        size_t ptr;
    public:
        BinaryReader(const uint8_t* _data, size_t _size) : data(_data), size(_size), ptr(0) {}
        
        int32_t ReadInt32() {
            if (ptr + 4 > size) return 0;
            int32_t v;
            std::memcpy(&v, data + ptr, 4);
            ptr += 4;
            return v;
        }
        TTC::Math::Fixed32 ReadFixed() {
            return TTC::Math::Fixed32::FromRaw(ReadInt32());
        }
        TTC::Math::Vector2 ReadVector2() {
            TTC::Math::Fixed32 x = ReadFixed();
            TTC::Math::Fixed32 y = ReadFixed();
            return TTC::Math::Vector2(x, y);
        }
        std::string ReadString() {
            int32_t len = ReadInt32();
            if (ptr + len > size || len < 0) return "";
            std::string s((const char*)(data + ptr), len);
            ptr += len;
            return s;
        }
        bool Eof() const { return ptr >= size; }
    };
}
}

namespace TTC {
namespace Physics {

    using namespace TTC::Math;

    struct AABB {
        Vector2 min;
        Vector2 max;

        AABB() : min(Vector2::Zero()), max(Vector2::Zero()) {}
        AABB(Vector2 _min, Vector2 _max) : min(_min), max(_max) {}

        bool Intersects(const AABB& other) const {
            if (max.x < other.min.x || min.x > other.max.x) return false;
            if (max.y < other.min.y || min.y > other.max.y) return false;
            return true;
        }

        bool Contains(const Vector2& point) const {
            return point.x >= min.x && point.x <= max.x && point.y >= min.y && point.y <= max.y;
        }
        
        static AABB FromCenterSize(Vector2 center, Vector2 size) {
            Vector2 half = size / Fixed32::FromInt(2);
            return AABB(center - half, center + half);
        }
    };

    struct Collider {
        TTC::Core::EntityID owner;
        AABB bounds;
        bool isStatic;
        uint32_t layerMask;
    };

    class QuadTree {
    private:
        static constexpr int MAX_OBJECTS = 8;
        static constexpr int MAX_LEVELS = 5;

        struct Node {
            int level;
            std::vector<Collider*> objects;
            AABB bounds;
            Node* nodes[4];

            Node(int lvl, AABB b) : level(lvl), bounds(b) {
                for(int i=0; i<4; i++) nodes[i] = nullptr;
            }
            
            ~Node() {
                for(int i=0; i<4; i++) if(nodes[i]) delete nodes[i];
            }
        };

        Node* root;
        std::vector<Collider*> allColliders;

        void Split(Node* node) {
            Vector2 subSize = (node->bounds.max - node->bounds.min) / Fixed32::FromInt(2);
            Fixed32 x = node->bounds.min.x;
            Fixed32 y = node->bounds.min.y;

            node->nodes[0] = new Node(node->level + 1, AABB(Vector2(x + subSize.x, y), Vector2(x + subSize.x * Fixed32::FromInt(2), y + subSize.y)));
            node->nodes[1] = new Node(node->level + 1, AABB(Vector2(x, y), Vector2(x + subSize.x, y + subSize.y)));
            node->nodes[2] = new Node(node->level + 1, AABB(Vector2(x, y + subSize.y), Vector2(x + subSize.x, y + subSize.y * Fixed32::FromInt(2))));
            node->nodes[3] = new Node(node->level + 1, AABB(Vector2(x + subSize.x, y + subSize.y), Vector2(x + subSize.x * Fixed32::FromInt(2), y + subSize.y * Fixed32::FromInt(2))));
        }

        int GetIndex(Node* node, const AABB& pRect) {
            int index = -1;
            Vector2 center = node->bounds.min + (node->bounds.max - node->bounds.min) / Fixed32::FromInt(2);
            bool topQuadrant = (pRect.min.y < center.y && pRect.max.y < center.y);
            bool bottomQuadrant = (pRect.min.y > center.y);

            if (pRect.min.x < center.x && pRect.max.x < center.x) {
                if (topQuadrant) index = 1;
                else if (bottomQuadrant) index = 2;
            }
            else if (pRect.min.x > center.x) {
                if (topQuadrant) index = 0;
                else if (bottomQuadrant) index = 3;
            }
            return index;
        }

        void Insert(Node* node, Collider* pRect) {
            if (node->nodes[0] != nullptr) {
                int index = GetIndex(node, pRect->bounds);
                if (index != -1) {
                    Insert(node->nodes[index], pRect);
                    return;
                }
            }

            node->objects.push_back(pRect);

            if (node->objects.size() > MAX_OBJECTS && node->level < MAX_LEVELS) {
                if (node->nodes[0] == nullptr) Split(node);
                int i = 0;
                while (i < node->objects.size()) {
                    int index = GetIndex(node, node->objects[i]->bounds);
                    if (index != -1) {
                        Collider* c = node->objects[i];
                        node->objects.erase(node->objects.begin() + i);
                        Insert(node->nodes[index], c);
                    } else {
                        i++;
                    }
                }
            }
        }

        void Retrieve(std::vector<Collider*>& returnObjects, Node* node, const AABB& pRect) {
            int index = GetIndex(node, pRect);
            if (index != -1 && node->nodes[0] != nullptr) {
                Retrieve(returnObjects, node->nodes[index], pRect);
            }
            returnObjects.insert(returnObjects.end(), node->objects.begin(), node->objects.end());
        }

    public:
        QuadTree(AABB bounds) {
            root = new Node(0, bounds);
        }

        ~QuadTree() {
            delete root;
        }

        void Clear() {
            allColliders.clear();
            delete root;
            root = new Node(0, AABB(Vector2(Fixed32::FromInt(-1000), Fixed32::FromInt(-1000)), Vector2(Fixed32::FromInt(1000), Fixed32::FromInt(1000))));
        }

        void AddCollider(Collider* c) {
            allColliders.push_back(c);
            Insert(root, c);
        }

        std::vector<Collider*> Query(const AABB& area) {
            std::vector<Collider*> result;
            Retrieve(result, root, area);
            return result;
        }
    };
}
}

namespace TTC {
namespace AI {

    class IState {
    public:
        virtual ~IState() = default;
        virtual void OnEnter() = 0;
        virtual void OnUpdate(TTC::Math::Fixed32 dt) = 0;
        virtual void OnExit() = 0;
    };

    class StateMachine {
    private:
        IState* currentState;
        std::map<std::string, IState*> states;

    public:
        StateMachine() : currentState(nullptr) {}
        
        ~StateMachine() {
            for(auto& pair : states) delete pair.second;
            states.clear();
        }

        void AddState(const std::string& name, IState* state) {
            states[name] = state;
        }

        void ChangeState(const std::string& name) {
            if (states.find(name) == states.end()) return;
            if (currentState) currentState->OnExit();
            currentState = states[name];
            currentState->OnEnter();
        }

        void Update(TTC::Math::Fixed32 dt) {
            if (currentState) currentState->OnUpdate(dt);
        }
    };
}
}

namespace TTC {
namespace Events {

    class IEvent {
    public:
        virtual ~IEvent() = default;
    };

    struct CollisionEvent : public IEvent {
        TTC::Core::EntityID a;
        TTC::Core::EntityID b;
        CollisionEvent(TTC::Core::EntityID _a, TTC::Core::EntityID _b) : a(_a), b(_b) {}
    };

    class EventBus {
    private:
        using HandlerFunc = std::function<void(const IEvent&)>;
        std::map<size_t, std::vector<HandlerFunc>> handlers;

    public:
        template<typename EventType>
        void Subscribe(std::function<void(const EventType&)> handler) {
            size_t typeId = typeid(EventType).hash_code();
            handlers[typeId].push_back([handler](const IEvent& e) {
                handler(static_cast<const EventType&>(e));
            });
        }

        template<typename EventType>
        void Publish(const EventType& event) {
            size_t typeId = typeid(EventType).hash_code();
            if (handlers.find(typeId) != handlers.end()) {
                for (auto& handler : handlers[typeId]) {
                    handler(event);
                }
            }
        }
    };

    extern EventBus* GEventBus;
}
}

#endif
