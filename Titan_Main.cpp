#include "Titan_Engine.hpp"
#include <cstdio>

using namespace Titan;

int main() {
    printf("[TITAN ENGINE v24.1] Booting...\n");

    Engine::Time::Init();
    Engine::ECS::Registry::Init();
    Debug::Menu::Init();

    Memory::LinearArena frameArena;
    Memory::LinearArena::Control::Init(frameArena, TITAN_MB(16));

    Debug::Menu::Create(); 

    Math::Vec3 pos = {0, 10, 0};
    Math::Vec3 velocity = {0, -9.8f, 0};
    f32 dt = 0.016f;

    Engine::ECS::EntityID player = Engine::ECS::Registry::CreateEntity();
    printf("Created Entity ID: %d\n", player);

    Data::String name = Data::String::Ops::Create("Player_One", &frameArena);
    u32 hash = Data::Hash::FNV1a_32(name.ptr);
    printf("Entity Name: %s | Hash: %u\n", name.ptr, hash);

    for(int i=0; i<10; ++i) {
        Debug::Menu::BeginFrame();

        Math::Vec3 gravityStep = Math::Vec3::Ops::Mul(velocity, dt);
        pos = Math::Vec3::Ops::Add(pos, gravityStep);

        Math::Geometry::Sphere s1 = { {0,0,0}, 1.0f };
        Math::Geometry::Sphere s2 = { pos, 0.5f };
        
        bool hit = Math::Geometry::Check::IntersectSphere(s1, s2);
        
        printf("Frame %d: Pos(%.2f, %.2f, %.2f) HitGround: %s\n", 
            i, pos.x, pos.y, pos.z, hit ? "YES" : "NO");
            
        Debug::Menu::EndFrame();
    }

    Debug::Menu::Shutdown();
    printf("Shutting down.\n");
    return 0;
}
