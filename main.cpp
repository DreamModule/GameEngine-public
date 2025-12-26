#include "TTC_Titan.hpp"
#include <cstdio>
#include <chrono>

using namespace TTC;

namespace TTC::Memory {
    thread_local ScratchArena g_TLSArena;
}

struct PhysicsData {
    ECS::Chunk** chunks;
    u32 count;
    f32 dt;
};

void PhysicsWorker(void* raw, void*) {
    auto* work = (PhysicsData*)raw;
    __m256 vDT = _mm256_set1_ps(work->dt);

    for(u32 c = 0; c < work->count; ++c) {
        ECS::Chunk* chunk = work->chunks[c];
        u16 count = chunk->header.count;

        // Assumes Layout: [Pos X][Pos Y][Pos Z] ... [Vel X][Vel Y][Vel Z]
        f32* px = (f32*)chunk->data;
        f32* py = px + 1024;
        f32* pz = py + 1024;
        f32* vx = pz + 1024;
        f32* vy = vx + 1024;
        f32* vz = vy + 1024;

        for (u32 i = 0; i < count; i += 8) {
            __m256 x = _mm256_loadu_ps(px + i);
            __m256 y = _mm256_loadu_ps(py + i);
            __m256 z = _mm256_loadu_ps(pz + i);
            __m256 dx = _mm256_loadu_ps(vx + i);
            __m256 dy = _mm256_loadu_ps(vy + i);
            __m256 dz = _mm256_loadu_ps(vz + i);

            x = _mm256_fmadd_ps(dx, vDT, x);
            y = _mm256_fmadd_ps(dy, vDT, y);
            z = _mm256_fmadd_ps(dz, vDT, z);

            _mm256_storeu_ps(px + i, x);
            _mm256_storeu_ps(py + i, y);
            _mm256_storeu_ps(pz + i, z);
        }
    }
}

int main() {
    printf("[TITAN ENGINE] Booting...\n");

    Job::Scheduler scheduler;
    std::thread workers[Job::Scheduler::MAX_THREADS];
    u32 threadCount = std::thread::hardware_concurrency();
    if(threadCount > Job::Scheduler::MAX_THREADS) threadCount = Job::Scheduler::MAX_THREADS;

    for(u32 i=0; i<threadCount; ++i) {
        workers[i] = std::thread([&scheduler, i]() {
            Memory::g_TLSArena.Init(TTC_MB(4));
            scheduler.WorkerLoop(i);
        });
    }

    Memory::g_TLSArena.Init(TTC_MB(4));

    ECS::World world;
    world.Init();

    ECS::Archetype* arch = (ECS::Archetype*)OS::VirtualReserve(sizeof(ECS::Archetype));
    OS::VirtualCommit(arch, sizeof(ECS::Archetype));
    arch->id = 0;
    arch->compCount = 6;
    world.archetypes.Push(arch);

    printf("  > Spawning 100,000 Entities...\n");
    for(int i=0; i<100; ++i) {
        ECS::Chunk* c = (ECS::Chunk*)OS::VirtualReserve(ECS::CHUNK_SIZE);
        OS::VirtualCommit(c, ECS::CHUNK_SIZE);
        c->header.count = 1000;
        arch->chunks.Push(c);
    }

    using Clock = std::chrono::high_resolution_clock;
    int frames = 0;

    while (frames < 600) {
        auto start = Clock::now();
        Memory::g_TLSArena.Reset();

        std::atomic<i32> jobCounter{10};
        for(int j=0; j<10; ++j) {
            auto* task = (PhysicsData*)Memory::g_TLSArena.Alloc(sizeof(PhysicsData));
            task->chunks = &arch->chunks.data[j * 10];
            task->count = 10;
            task->dt = 0.016f;

            Job::Job job;
            job.function = PhysicsWorker;
            job.data = task;
            job.counter = &jobCounter;
            scheduler.Schedule(job, 0);
        }

        while(jobCounter.load(std::memory_order_acquire) > 0) {
            _mm_pause();
        }

        auto end = Clock::now();
        std::chrono::duration<f64, std::milli> dt = end - start;
        frames++;

        if (frames % 60 == 0) {
             printf("  Frame: %d | Time: %.4f ms | Entities: 100k\n", frames, dt.count());
        }
    }

    scheduler.running = false;
    for(u32 i=0; i<threadCount; ++i) workers[i].join();
    return 0;
}
