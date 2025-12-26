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

#endif
