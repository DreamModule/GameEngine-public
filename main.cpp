//3D snake game on this engine 
#include "TTC_Engine_Monolith.hpp"
#include <thread>
#include <random>
#include <iostream>
#include <vector>

#ifdef _WIN32
#include <conio.h>
#endif

namespace TTC {
    namespace Memory { IAllocator* GlobalAllocator = nullptr; }
    namespace Input { InputManager* GInput = nullptr; }
    namespace Time { Clock* GClock = nullptr; }
    namespace Debug { DebugLayer* GDebug = nullptr; }
    namespace Core { EngineContext* GEngine = nullptr; }
    namespace Events { EventBus* GEventBus = nullptr; }
}

struct Transform {
    TTC::Math::Vector3 position;
};

struct SnakeSegment {
    int index;
};

struct Food {
    TTC::Math::Fixed32 value;
};

class Snake3DGame {
private:
    TTC::Core::Registry registry;
    TTC::Math::Vector3 currentDir;
    TTC::Math::Vector3 nextDir;
    TTC::Math::Fixed32 moveTimer;
    TTC::Math::Fixed32 moveInterval;
    
    TTC::Core::EntityID headEntity;
    std::vector<TTC::Core::EntityID> bodyEntities;
    TTC::Core::EntityID foodEntity;

    int gridSize;
    bool isGameOver;
    int score;

    std::mt19937 rng;

public:
    Snake3DGame() : rng(std::random_device{}()) {
        InitGlobals();
        gridSize = 10;
        Restart();
    }

    ~Snake3DGame() {
        CleanupGlobals();
    }

    void InitGlobals() {
        TTC::Memory::GlobalAllocator = new TTC::Memory::HeapAllocator();
        TTC::Core::GEngine = new TTC::Core::EngineContext();
        TTC::Core::GEngine->Initialize();
        TTC::Input::GInput = new TTC::Input::InputManager();
        TTC::Time::GClock = new TTC::Time::Clock();
        TTC::Debug::GDebug = new TTC::Debug::DebugLayer();
    }

    void CleanupGlobals() {
        delete TTC::Debug::GDebug;
        delete TTC::Time::GClock;
        delete TTC::Input::GInput;
        delete TTC::Core::GEngine;
        delete TTC::Memory::GlobalAllocator;
    }

    void Restart() {
        currentDir = TTC::Math::Vector3::Right(); 
        nextDir = currentDir;
        moveTimer = TTC::Math::Fixed32::Zero();
        moveInterval = TTC::Math::Fixed32::FromFloat(0.3f); 
        isGameOver = false;
        score = 0;
        bodyEntities.clear();

        headEntity = registry.Create();
        Transform tHead;
        tHead.position = TTC::Math::Vector3::Zero(); 
        registry.Emplace<Transform>(headEntity, tHead);
        registry.Emplace<SnakeSegment>(headEntity, { 0 });

        SpawnFood();
    }

    void SpawnFood() {
        std::uniform_int_distribution<int> dist(-gridSize/2, gridSize/2);
        int x = dist(rng);
        int z = dist(rng);
        
        foodEntity = registry.Create();
        Transform tFood;
        tFood.position = TTC::Math::Vector3(
            TTC::Math::Fixed32::FromInt(x),
            TTC::Math::Fixed32::Zero(),
            TTC::Math::Fixed32::FromInt(z)
        );
        registry.Emplace<Transform>(foodEntity, tFood);
        registry.Emplace<Food>(foodEntity, { TTC::Math::Fixed32::FromInt(10) });
    }

    void Grow() {
        TTC::Core::EntityID tailId = (bodyEntities.empty()) ? headEntity : bodyEntities.back();
        Transform tailTrans = registry.Get<Transform>(tailId);

        TTC::Core::EntityID newSeg = registry.Create();
        registry.Emplace<Transform>(newSeg, tailTrans); 
        registry.Emplace<SnakeSegment>(newSeg, { (int)bodyEntities.size() + 1 });
        
        bodyEntities.push_back(newSeg);
        score += 100;
        
        if (moveInterval > TTC::Math::Fixed32::FromFloat(0.05f)) {
            moveInterval -= TTC::Math::Fixed32::FromFloat(0.01f);
        }
    }

    void PlatformInput() {
#ifdef _WIN32
        if (_kbhit()) {
            char ch = _getch();
            if (ch == 'w') TTC::Input::GInput->ProcessEvent(TTC::Input::Key::ArrowUp, true);
            if (ch == 's') TTC::Input::GInput->ProcessEvent(TTC::Input::Key::ArrowDown, true);
            if (ch == 'a') TTC::Input::GInput->ProcessEvent(TTC::Input::Key::ArrowLeft, true);
            if (ch == 'd') TTC::Input::GInput->ProcessEvent(TTC::Input::Key::ArrowRight, true);
        } else {
             TTC::Input::GInput->ProcessEvent(TTC::Input::Key::ArrowUp, false);
             TTC::Input::GInput->ProcessEvent(TTC::Input::Key::ArrowDown, false);
             TTC::Input::GInput->ProcessEvent(TTC::Input::Key::ArrowLeft, false);
             TTC::Input::GInput->ProcessEvent(TTC::Input::Key::ArrowRight, false);
        }
#endif
    }

    void HandleInput() {
        using namespace TTC::Input;
        using namespace TTC::Math;

        PlatformInput();
        GInput->Update(); 

        if (GInput->GetKeyDown(Key::ArrowUp) && currentDir.z == Fixed32::Zero()) {
            nextDir = Vector3::Forward(); 
        }
        else if (GInput->GetKeyDown(Key::ArrowDown) && currentDir.z == Fixed32::Zero()) {
            nextDir = Vector3::Forward() * Fixed32::FromInt(-1); 
        }
        else if (GInput->GetKeyDown(Key::ArrowLeft) && currentDir.x == Fixed32::Zero()) {
            nextDir = Vector3::Right() * Fixed32::FromInt(-1); 
        }
        else if (GInput->GetKeyDown(Key::ArrowRight) && currentDir.x == Fixed32::Zero()) {
            nextDir = Vector3::Right(); 
        }
    }

    void UpdatePhysics() {
        using namespace TTC::Math;

        moveTimer += TTC::Time::GClock->GetFixedDeltaTime();

        if (moveTimer >= moveInterval) {
            moveTimer = Fixed32::Zero();
            currentDir = nextDir;

            if (!bodyEntities.empty()) {
                for (size_t i = bodyEntities.size() - 1; i > 0; --i) {
                    Transform& curr = registry.Get<Transform>(bodyEntities[i]);
                    Transform& prev = registry.Get<Transform>(bodyEntities[i-1]);
                    curr.position = prev.position;
                }
                Transform& firstBody = registry.Get<Transform>(bodyEntities[0]);
                Transform& headTrans = registry.Get<Transform>(headEntity);
                firstBody.position = headTrans.position;
            }

            Transform& head = registry.Get<Transform>(headEntity);
            head.position = head.position + currentDir;

            int hx = head.position.x.ToInt();
            int hz = head.position.z.ToInt();
            int limit = gridSize / 2;

            if (hx > limit || hx < -limit || hz > limit || hz < -limit) {
                isGameOver = true;
                return;
            }

            for (auto id : bodyEntities) {
                Transform& bodyPart = registry.Get<Transform>(id);
                if (Vector3::Distance(head.position, bodyPart.position) < Fixed32::FromFloat(0.1f)) {
                    isGameOver = true;
                    return;
                }
            }

            Transform& foodTrans = registry.Get<Transform>(foodEntity);
            if (Vector3::Distance(head.position, foodTrans.position) < Fixed32::FromFloat(0.1f)) {
                Grow();
                SpawnFood(); 
            }
        }
    }

    void DrawDebugUI() {
        using namespace TTC::Debug;
        using namespace TTC::Math;

        GDebug->Draw([&]() {
            if (GDebug->BeginWindow("Snake 3D Debug", Vector2::FromInts(10, 10), Vector2::FromInts(300, 400))) {
                
                GDebug->Text("FPS: 60 (Simulated)");
                GDebug->Separator();
                
                if (isGameOver) {
                    GDebug->Text("GAME OVER!");
                    GDebug->Text("Final Score: %d", score);
                    if (GDebug->Button("Restart")) {
                        Restart();
                    }
                } else {
                    GDebug->Text("Score: %d", score);
                    GDebug->Text("Head Pos: %s", registry.Get<Transform>(headEntity).position.x.ToString().c_str()); 
                    
                    Fixed32 speed = moveInterval;
                    GDebug->SliderFixed("Tick Rate", &moveInterval, Fixed32::FromFloat(0.05f), Fixed32::FromFloat(1.0f));
                }

                GDebug->Separator();
                GDebug->Text("Map Visualization (XZ Plane):");
                
                int limit = gridSize / 2;
                Transform& hT = registry.Get<Transform>(headEntity);
                Transform& fT = registry.Get<Transform>(foodEntity);

                std::string line;
                for (int z = limit; z >= -limit; --z) {
                    line = "";
                    for (int x = -limit; x <= limit; ++x) {
                        bool drawn = false;
                        
                        if (hT.position.x.ToInt() == x && hT.position.z.ToInt() == z) {
                            line += "O"; drawn = true;
                        } 
                        else if (fT.position.x.ToInt() == x && fT.position.z.ToInt() == z) {
                            line += "X"; drawn = true;
                        }
                        else {
                            for(auto id : bodyEntities) {
                                Transform& bT = registry.Get<Transform>(id);
                                if (bT.position.x.ToInt() == x && bT.position.z.ToInt() == z) {
                                    line += "o"; drawn = true; break;
                                }
                            }
                        }

                        if (!drawn) line += ".";
                        line += " ";
                    }
                    GDebug->Text("%s", line.c_str());
                }
            }
            GDebug->EndWindow();
        });
    }

    void RenderConsole() {
        #ifdef _WIN32
        system("cls");
        #else
        system("clear");
        #endif

        std::cout << "=== TTC ENGINE SNAKE 3D ===" << std::endl;
        std::cout << "Score: " << score << std::endl;
        if(isGameOver) std::cout << " [ GAME OVER ] " << std::endl;

        int limit = gridSize / 2;
        Transform& hT = registry.Get<Transform>(headEntity);
        Transform& fT = registry.Get<Transform>(foodEntity);

        for (int z = limit; z >= -limit; --z) {
            for (int x = -limit; x <= limit; ++x) {
                char c = '.';
                
                if (hT.position.x.ToInt() == x && hT.position.z.ToInt() == z) c = 'H';
                else if (fT.position.x.ToInt() == x && fT.position.z.ToInt() == z) c = 'F';
                else {
                    for(auto id : bodyEntities) {
                        Transform& bT = registry.Get<Transform>(id);
                        if (bT.position.x.ToInt() == x && bT.position.z.ToInt() == z) {
                            c = 'o'; break;
                        }
                    }
                }
                std::cout << c << " ";
            }
            std::cout << std::endl;
        }
        std::cout << "Controls: WASD (English Layout)" << std::endl;
    }

    void Run() {
        while (true) {
            TTC::Core::GEngine->Update();
            
            if (!isGameOver) {
                HandleInput();
                UpdatePhysics();
            }

            DrawDebugUI();
            RenderConsole();

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
};

int main() {
    Snake3DGame game;
    game.Run();
    return 0;
}
