// Rigid body docking simulation: a moving chaser cube hits a stationary target
// cube. Physics runs on its own thread at a fixed step (default 1000 us = 1 kHz,
// set with --dt-us), rendering at ~60 FPS, and the data is logged to
// simulation.log. Keys: 1/2 scenario, R reset, SPACE pause.
// Runs on Windows and Linux (Ubuntu). See --help for options.

#include <cmath>
#include <iostream>
#include <vector>
#include <iomanip>
#include <string>
#include <algorithm>

#include <thread>
#include <atomic>
#include <fstream>
#include <sstream>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <mmsystem.h>
#elif defined(__linux__)
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#endif

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#include <immintrin.h>
#endif


#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"


// ============================================================
// 1. VECTOR
// 3D vector with basic math (add, subtract, scale, dot, cross, length).
// ============================================================
struct Vec3
{
    double x, y, z;

    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
    Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }

    Vec3& operator+=(const Vec3& o)
    {
        x += o.x; y += o.y; z += o.z;
        return *this;
    }

    double Dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }

    Vec3 Cross(const Vec3& o) const
    {
        return Vec3(
            y * o.z - z * o.y,
            z * o.x - x * o.z,
            x * o.y - y * o.x
        );
    }

    double Magnitude() const { return std::sqrt(Dot(*this)); }
};


// ============================================================
// 2. QUATERNION
// Stores a 3D orientation and can combine, normalise and convert rotations.
// ============================================================
struct Quat
{
    double w, x, y, z;

    Quat(double w = 1.0, double x = 0.0, double y = 0.0, double z = 0.0)
        : w(w), x(x), y(y), z(z) {}

    Quat operator*(const Quat& q) const
    {
        return Quat(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        );
    }

    Quat Normalized() const
    {
        double magnitude = std::sqrt(w * w + x * x + y * y + z * z);

        if (magnitude < 1e-12)
            return Quat(1.0, 0.0, 0.0, 0.0);

        return Quat(w / magnitude, x / magnitude, y / magnitude, z / magnitude);
    }

    static Quat FromAngularDisplacement(const Vec3& deltaTheta)
    {
        double angle = deltaTheta.Magnitude();

        if (angle < 1e-12)
            return Quat(1.0, 0.0, 0.0, 0.0);

        double halfAngle = angle * 0.5;
        double sinHalf = std::sin(halfAngle);

        return Quat(
            std::cos(halfAngle),
            (deltaTheta.x / angle) * sinHalf,
            (deltaTheta.y / angle) * sinHalf,
            (deltaTheta.z / angle) * sinHalf
        );
    }

    glm::mat4 ToRotationMatrix() const
    {
        glm::mat4 m(1.0f);

        m[0][0] = static_cast<float>(1.0 - 2.0 * (y * y + z * z));
        m[0][1] = static_cast<float>(2.0 * (x * y + w * z));
        m[0][2] = static_cast<float>(2.0 * (x * z - w * y));

        m[1][0] = static_cast<float>(2.0 * (x * y - w * z));
        m[1][1] = static_cast<float>(1.0 - 2.0 * (x * x + z * z));
        m[1][2] = static_cast<float>(2.0 * (y * z + w * x));

        m[2][0] = static_cast<float>(2.0 * (x * z + w * y));
        m[2][1] = static_cast<float>(2.0 * (y * z - w * x));
        m[2][2] = static_cast<float>(1.0 - 2.0 * (x * x + y * y));

        return m;
    }
};


// ============================================================
// 3. RIGID BODY
// Holds the mass, inertia, position, velocity, orientation and size of a cube.
// ============================================================
struct RigidBody
{
    std::string name;
    double mass;
    Vec3 inertia;
    Vec3 position;
    Vec3 velocity;
    Quat attitude;
    Vec3 angularVel;
    Vec3 size;
};


// ============================================================
// 4. LOAD CELL POSITIONS
// Positions of the 6 force sensors on the target's contact face (hexagon).
// ============================================================
const Vec3 g_loadCellPos[6] =
{
    Vec3(0.0,  0.50,  0.000),
    Vec3(0.0,  0.25,  0.433),
    Vec3(0.0, -0.25,  0.433),
    Vec3(0.0, -0.50,  0.000),
    Vec3(0.0, -0.25, -0.433),
    Vec3(0.0,  0.25, -0.433)
};


// ============================================================
// 5. CONTACT FORCE DISTRIBUTION
// Splits the total contact force into 6 load cell readings.
// ============================================================
void DistributeContactForceToLoadCells(
    const Vec3& totalContactForce,
    const Vec3& contactPt,
    Vec3 outForces[6])
{
    Vec3 torque = contactPt.Cross(totalContactForce);

    double sumY2 = 0.0;
    double sumZ2 = 0.0;

    for (int i = 0; i < 6; ++i)
    {
        sumY2 += g_loadCellPos[i].y * g_loadCellPos[i].y;
        sumZ2 += g_loadCellPos[i].z * g_loadCellPos[i].z;
    }

    for (int i = 0; i < 6; ++i)
    {
        double fNormal =
            totalContactForce.x / 6.0
            + torque.y * g_loadCellPos[i].z / sumZ2
            - torque.z * g_loadCellPos[i].y / sumY2;

        double fShearY = totalContactForce.y / 6.0;
        double fShearZ = totalContactForce.z / 6.0;

        outForces[i] = Vec3(fNormal, fShearY, fShearZ);
    }
}


// ============================================================
// 6. CONTACT ESTIMATION
// From the 6 load cell forces, computes net force/torque, contact point,
// accelerations and the displacement over one time step.
// ============================================================
struct ContactEstimationResult
{
    Vec3 F_net;
    Vec3 tau_net;
    Vec3 Pc_est;
    Vec3 a_CM;
    Vec3 alpha;
    Vec3 delta_x;
    Vec3 delta_theta;
};


ContactEstimationResult ComputePostContactPose(
    const Vec3 forces[6],
    double mass,
    const Vec3& inertia,
    double dt)
{
    ContactEstimationResult result;

    result.F_net = Vec3(0, 0, 0);
    for (int i = 0; i < 6; ++i)
        result.F_net += forces[i];

    result.tau_net = Vec3(0, 0, 0);
    for (int i = 0; i < 6; ++i)
        result.tau_net += g_loadCellPos[i].Cross(forces[i]);

    double forceSquared = result.F_net.Dot(result.F_net);

    if (forceSquared > 1e-12)
        result.Pc_est = result.F_net.Cross(result.tau_net) * (1.0 / forceSquared);
    else
        result.Pc_est = Vec3(0, 0, 0);

    result.a_CM = result.F_net * (1.0 / mass);

    result.alpha = Vec3(
        result.tau_net.x / inertia.x,
        result.tau_net.y / inertia.y,
        result.tau_net.z / inertia.z
    );

    double dtSquaredHalf = 0.5 * dt * dt;

    result.delta_x = result.a_CM * dtSquaredHalf;
    result.delta_theta = result.alpha * dtSquaredHalf;

    return result;
}


// ============================================================
// 7. SHADERS
// GPU programs that position the vertices and apply simple lighting.
// ============================================================
const char* vertexShaderSource = R"(
#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

uniform mat4 uMVP;
uniform mat4 uModel;

out vec3 FragPos;
out vec3 Normal;

void main()
{
    FragPos = vec3(uModel * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(uModel))) * aNormal;
    gl_Position = uMVP * vec4(aPos, 1.0);
}
)";


const char* fragmentShaderSource = R"(
#version 330 core

in vec3 FragPos;
in vec3 Normal;

uniform vec3 uColor;

out vec4 FragColor;

void main()
{
    vec3 lightDir = normalize(vec3(2.0, 3.0, 2.5));
    vec3 normal = normalize(Normal);

    float diffuse = max(dot(normal, lightDir), 0.0);

    vec3 ambient = 0.35 * uColor;
    vec3 diffuseColor = diffuse * uColor * 0.85;

    FragColor = vec4(ambient + diffuseColor, 1.0);
}
)";


// ============================================================
// 8. CREATE SHADER PROGRAM
// Compiles and links the shaders; prints any errors and returns 0 on failure.
// ============================================================
static GLuint CompileShader(GLenum type, const char* source)
{
    GLuint shader = glCreateShader(type);

    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint ok = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);

    if (!ok)
    {
        char log[1024];
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        std::cerr << "Shader compile error:\n" << log << "\n";
        glDeleteShader(shader);
        return 0;
    }

    return shader;
}


GLuint CreateShaderProgram()
{
    GLuint vertexShader = CompileShader(GL_VERTEX_SHADER, vertexShaderSource);
    GLuint fragmentShader = CompileShader(GL_FRAGMENT_SHADER, fragmentShaderSource);

    if (!vertexShader || !fragmentShader)
    {
        if (vertexShader) glDeleteShader(vertexShader);
        if (fragmentShader) glDeleteShader(fragmentShader);
        return 0;
    }

    GLuint program = glCreateProgram();

    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    glLinkProgram(program);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    GLint ok = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);

    if (!ok)
    {
        char log[1024];
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        std::cerr << "Shader link error:\n" << log << "\n";
        glDeleteProgram(program);
        return 0;
    }

    return program;
}


// ============================================================
// 9. CREATE CUBE
// Uploads a unit cube mesh (positions + normals) to the GPU.
// It is reused for both cubes and the floor.
// ============================================================
void CreateCubeMesh(GLuint& vao, GLuint& vbo)
{
    float vertices[] =
    {
        // Position              Normal

        // Front
        -0.5f,-0.5f,-0.5f,       0,0,-1,
         0.5f,-0.5f,-0.5f,       0,0,-1,
         0.5f, 0.5f,-0.5f,       0,0,-1,

         0.5f, 0.5f,-0.5f,       0,0,-1,
        -0.5f, 0.5f,-0.5f,       0,0,-1,
        -0.5f,-0.5f,-0.5f,       0,0,-1,

        // Back
        -0.5f,-0.5f, 0.5f,       0,0,1,
         0.5f,-0.5f, 0.5f,       0,0,1,
         0.5f, 0.5f, 0.5f,       0,0,1,

         0.5f, 0.5f, 0.5f,       0,0,1,
        -0.5f, 0.5f, 0.5f,       0,0,1,
        -0.5f,-0.5f, 0.5f,       0,0,1,

        // Left
        -0.5f, 0.5f, 0.5f,      -1,0,0,
        -0.5f, 0.5f,-0.5f,      -1,0,0,
        -0.5f,-0.5f,-0.5f,      -1,0,0,

        -0.5f,-0.5f,-0.5f,      -1,0,0,
        -0.5f,-0.5f, 0.5f,      -1,0,0,
        -0.5f, 0.5f, 0.5f,      -1,0,0,

        // Right
         0.5f, 0.5f, 0.5f,       1,0,0,
         0.5f, 0.5f,-0.5f,       1,0,0,
         0.5f,-0.5f,-0.5f,       1,0,0,

         0.5f,-0.5f,-0.5f,       1,0,0,
         0.5f,-0.5f, 0.5f,       1,0,0,
         0.5f, 0.5f, 0.5f,       1,0,0,

        // Bottom
        -0.5f,-0.5f,-0.5f,       0,-1,0,
         0.5f,-0.5f,-0.5f,       0,-1,0,
         0.5f,-0.5f, 0.5f,       0,-1,0,

         0.5f,-0.5f, 0.5f,       0,-1,0,
        -0.5f,-0.5f, 0.5f,       0,-1,0,
        -0.5f,-0.5f,-0.5f,       0,-1,0,

        // Top
        -0.5f, 0.5f,-0.5f,       0,1,0,
         0.5f, 0.5f,-0.5f,       0,1,0,
         0.5f, 0.5f, 0.5f,       0,1,0,

         0.5f, 0.5f, 0.5f,       0,1,0,
        -0.5f, 0.5f, 0.5f,       0,1,0,
        -0.5f, 0.5f,-0.5f,       0,1,0
    };

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}


// ============================================================
// 10. GLOBAL SIMULATION STATE
// The two scenarios, the two cubes and the simulation flags.
// ============================================================
enum class Scenario
{
    HeadOn = 1,
    OffCenter = 2
};

Scenario g_currentScenario = Scenario::HeadOn;

RigidBody g_target;
RigidBody g_chaser;

bool g_contactOccurred = false;
bool g_paused = false;


// ============================================================
// 11. COMMANDS
// Flags set by the key callback and read by the physics thread.
// ============================================================
std::atomic<int>  g_requestedScenario{1};
std::atomic<bool> g_resetRequested{false};
std::atomic<bool> g_pauseRequested{false};


// ============================================================
// 12. THREAD CONTROL
// Flags that tell the physics thread and logger thread when to stop.
// ============================================================
std::atomic<bool> g_running{true};
std::atomic<bool> g_loggerStop{false};
std::atomic<int>  g_pauseNotice{0};   // 1 = paused, 2 = resumed; printed by the render thread


// ============================================================
// 12b. RUNTIME CONFIG AND TIMING STATS
// Physics period is chosen at start-up (--dt-us). Stats are written by the
// physics thread and shown in the UI / printed on exit.
// ============================================================
constexpr int DEFAULT_PHYSICS_PERIOD_US = 1000;  // 1 kHz = 1 ms (was 2000 us)
constexpr int MIN_PHYSICS_PERIOD_US     = 50;
constexpr int MAX_PHYSICS_PERIOD_US     = 100000;

struct SimConfig
{
    int    physicsPeriodUs = DEFAULT_PHYSICS_PERIOD_US;
    bool   headless        = false;   // physics only, no window (benchmarks, CI, SSH)
    double headlessSeconds = 10.0;
    bool   secondsSpecified = false;
    bool   logEnabled      = true;
    bool   timingLogEnabled = true;   // timestep.csv: one row per physics tick
    int    physicsCpu      = -1;      // pin physics thread to this core (-1 = no pinning)
    bool   spin            = false;   // never sleep: busy-wait the whole period (lowest jitter)
    double tolUs           = 50.0;    // a tick starting later than this vs. its schedule is a violation
    int    logEvery        = 1;       // log every Nth physics step
    bool   realtime        = true;    // try to raise physics thread priority
    bool   spinForced      = false;   // user passed --spin or --no-spin explicitly
    bool   autoTune        = true;    // pick core + spin automatically when enough cores exist
    int    warmupMs        = 300;     // ignore start-up transients in the run statistics
};

SimConfig g_config;

constexpr int PERIOD_HIST_BUCKETS = 400;           // 10 us buckets, last = overflow

struct alignas(64) TimingStats
{
    // Live values (last ~1 s window) read by the UI
    std::atomic<uint32_t> windowAvgPeriodNs{0};
    std::atomic<uint32_t> windowMaxPeriodNs{0};
    std::atomic<uint32_t> windowMaxWorkNs{0};
    std::atomic<uint32_t> windowMaxLatencyNs{0};   // tick start vs. its scheduled time
    std::atomic<uint64_t> latencyViolations{0};    // latency > tolUs
    std::atomic<uint64_t> lateTicks{0};        // period > 1.5x target
    std::atomic<uint64_t> droppedLogRows{0};
    std::atomic<uint64_t> droppedTimingRows{0};
    std::atomic<uint64_t> workViolations{0};       // work >= 1000 us

    // Whole-run values, only read after the physics thread has joined
    uint64_t runSamples = 0;
    uint64_t runSumPeriodNs = 0;
    uint32_t runMaxPeriodNs = 0;
    uint32_t runMaxWorkNs = 0;
    uint32_t runMaxLatencyNs = 0;
    std::array<uint32_t, PERIOD_HIST_BUCKETS> periodHist{};
};

TimingStats g_timing;


// ============================================================
// 13. STATE PASSED TO RENDERER
// Snapshot of both cubes' pose sent from physics to rendering.
// ============================================================
struct SimulationState
{
    Vec3 cube1Position;
    Quat cube1Orientation;
    Vec3 cube1Velocity;
    Vec3 cube2Position;
    Quat cube2Orientation;
    Vec3 cube2Velocity;
    uint64_t physicsStep;
    bool contactOccurred;
    bool paused;
    int scenario;
};


// ============================================================
// 14. TRIPLE BUFFER
// Three state slots so physics and rendering can share data without blocking.
// ============================================================
struct alignas(64) StateBuffer
{
    SimulationState state;
    std::atomic<uint64_t> sequence{0};
};

alignas(64) StateBuffer g_stateBuffers[3];

int g_physicsBufferIndex = 0;


// ============================================================
// 15. LOGGING
// One record per physics tick is queued and written to file by the logger thread.
// ============================================================
struct LogRecord
{
    double timestamp;
    Vec3 cube1Position;
    Quat cube1Orientation;
    Vec3 cube2Position;
    Quat cube2Orientation;
    uint64_t physicsStep;
};

// Lock-free single-producer / single-consumer ring: the physics thread never
// takes a lock or allocates when logging. If the logger falls behind and the
// ring fills up, rows are dropped (and counted) instead of stalling physics.
template <typename T, size_t N>
class SpscRing
{
    static_assert((N & (N - 1)) == 0, "N must be a power of two");

public:
    bool Push(const T& v)
    {
        const size_t h = head_.load(std::memory_order_relaxed);
        if (h - tail_.load(std::memory_order_acquire) == N)
            return false;
        buf_[h & (N - 1)] = v;
        head_.store(h + 1, std::memory_order_release);
        return true;
    }

    // Touch every page once so the physics thread never takes a page fault
    void Prefault()
    {
        volatile unsigned char* p = reinterpret_cast<volatile unsigned char*>(buf_.data());
        for (size_t i = 0; i < sizeof(buf_); i += 4096)
            p[i] = p[i];
    }

    bool Pop(T& out)
    {
        const size_t t = tail_.load(std::memory_order_relaxed);
        if (t == head_.load(std::memory_order_acquire))
            return false;
        out = buf_[t & (N - 1)];
        tail_.store(t + 1, std::memory_order_release);
        return true;
    }

private:
    std::array<T, N> buf_{};
    alignas(64) std::atomic<size_t> head_{0};
    alignas(64) std::atomic<size_t> tail_{0};
};

SpscRing<LogRecord, (1u << 15)> g_logRing;     // ~16 s of buffer at 2 kHz

// One row per physics tick, written to timestep.csv so the real tick rate can
// be verified afterwards (see check_timing.py).
struct TimingRecord
{
    uint64_t step;
    int64_t  timestampNs;   // tick start, nanoseconds since physics thread start
    int64_t  periodNs;      // time since previous tick start (0 for the first tick)
    int64_t  latencyNs;     // tick start minus its scheduled start (scheduling delay)
};

SpscRing<TimingRecord, (1u << 16)> g_timingRing;


// ============================================================
// 16. WINDOW
// ============================================================
GLFWwindow* g_window = nullptr;
std::chrono::steady_clock::time_point g_guiStartTime;


// ============================================================
// 17. SETUP SCENARIO
// Resets both cubes to their starting state for the chosen scenario.
// ============================================================
void SetupScenario(Scenario scenario)
{
    g_currentScenario = scenario;
    g_contactOccurred = false;

    // Target
    g_target.name = "Target";
    g_target.mass = 50.0;
    g_target.inertia = Vec3(5.0, 5.0, 10.0);
    g_target.position = Vec3(0.0, 0.0, 0.0);
    g_target.velocity = Vec3(0.0, 0.0, 0.0);
    g_target.attitude = Quat(1.0, 0.0, 0.0, 0.0);
    g_target.angularVel = Vec3(0.0, 0.0, 0.0);
    g_target.size = Vec3(1.0, 1.0, 1.0);

    // Chaser
    g_chaser.name = "Chaser";
    g_chaser.mass = 50.0;
    g_chaser.inertia = Vec3(5.0, 5.0, 10.0);
    g_chaser.velocity = Vec3(0.5, 0.0, 0.0);
    g_chaser.attitude = Quat(1.0, 0.0, 0.0, 0.0);
    g_chaser.angularVel = Vec3(0.0, 0.0, 0.0);
    g_chaser.size = Vec3(1.0, 1.0, 1.0);

    if (scenario == Scenario::HeadOn)
    {
        g_chaser.position = Vec3(-3.0, 0.0, 0.0);
        std::cout << ">>> SCENARIO 1: HEAD-ON\n";
    }
    else
    {
        g_chaser.position = Vec3(-3.0, 0.35, 0.15);
        std::cout << ">>> SCENARIO 2: OFF-CENTER\n";
    }
}


// ============================================================
// 18. PROCESS COMMANDS
// Applies any pending reset or pause request (called every physics tick).
// ============================================================
void ProcessCommands()
{
    if (g_resetRequested.exchange(false))
    {
        int scenario = g_requestedScenario.load();

        if (scenario == 1)
            SetupScenario(Scenario::HeadOn);
        else
            SetupScenario(Scenario::OffCenter);
    }

    if (g_pauseRequested.exchange(false))
    {
        g_paused = !g_paused;

        // Console output can block, so it is not done on the physics thread
        g_pauseNotice.store(g_paused ? 1 : 2);
    }
}


// ============================================================
// 19. KEYBOARD CALLBACK
// Turns key presses into command flags (1/2 scenario, R reset, SPACE pause).
// ============================================================
void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action != GLFW_PRESS)
        return;

    if (key == GLFW_KEY_1)
    {
        g_requestedScenario = 1;
        g_resetRequested = true;
    }
    else if (key == GLFW_KEY_2)
    {
        g_requestedScenario = 2;
        g_resetRequested = true;
    }
    else if (key == GLFW_KEY_R)
    {
        g_resetRequested = true;
    }
    else if (key == GLFW_KEY_SPACE)
    {
        g_pauseRequested = true;
    }
}


// ============================================================
// 20. PUBLISH PHYSICS STATE
// Physics thread copies the current pose into the next triple-buffer slot.
// ============================================================
void PublishState(uint64_t physicsStep)
{
    StateBuffer& buffer = g_stateBuffers[g_physicsBufferIndex];

    uint64_t sequence = buffer.sequence.load(std::memory_order_relaxed);

    // Odd = being written
    buffer.sequence.store(sequence + 1, std::memory_order_relaxed);

    std::atomic_thread_fence(std::memory_order_release);

    buffer.state.cube1Position = g_target.position;
    buffer.state.cube1Orientation = g_target.attitude;
    buffer.state.cube1Velocity = g_target.velocity;
    buffer.state.cube2Position = g_chaser.position;
    buffer.state.cube2Orientation = g_chaser.attitude;
    buffer.state.cube2Velocity = g_chaser.velocity;
    buffer.state.physicsStep = physicsStep;
    buffer.state.contactOccurred = g_contactOccurred;
    buffer.state.paused = g_paused;
    buffer.state.scenario = static_cast<int>(g_currentScenario);

    // Even = complete
    buffer.sequence.store(sequence + 2, std::memory_order_release);

    g_physicsBufferIndex = (g_physicsBufferIndex + 1) % 3;
}


// ============================================================
// 21. GET LATEST COMPLETE STATE
// Render thread picks the newest fully written buffer.
// ============================================================
SimulationState GetLatestRenderState()
{
    // Reused if no buffer is usable this frame
    static SimulationState lastGood{};

    uint64_t bestStep = 0;
    bool found = false;

    for (int i = 0; i < 3; ++i)
    {
        StateBuffer& buffer = g_stateBuffers[i];

        uint64_t seq1 = buffer.sequence.load(std::memory_order_acquire);

        if (seq1 & 1)
            continue;

        SimulationState candidate = buffer.state;

        std::atomic_thread_fence(std::memory_order_acquire);

        uint64_t seq2 = buffer.sequence.load(std::memory_order_relaxed);

        if (seq1 != seq2)
            continue;

        if (!found || candidate.physicsStep > bestStep)
        {
            lastGood = candidate;
            bestStep = candidate.physicsStep;
            found = true;
        }
    }

    return lastGood;
}


// ============================================================
// 22. PHYSICS UPDATE
// One physics step: detect collision, apply the impact once, then move the cubes.
// ============================================================
void UpdatePhysics(double dt)
{
    if (g_paused)
        return;

    // Collision detection
    double overlapX =
        (g_chaser.position.x + g_chaser.size.x * 0.5)
        - (g_target.position.x - g_target.size.x * 0.5);

    double overlapY =
        (g_chaser.size.y * 0.5 + g_target.size.y * 0.5)
        - std::abs(g_chaser.position.y - g_target.position.y);

    double overlapZ =
        (g_chaser.size.z * 0.5 + g_target.size.z * 0.5)
        - std::abs(g_chaser.position.z - g_target.position.z);

    bool isTouching = overlapX >= 0.0 && overlapY > 0.0 && overlapZ > 0.0;

    // Contact response (happens once)
    if (!g_contactOccurred && isTouching)
    {
        g_contactOccurred = true;

        double contactY = (g_chaser.position.y + g_target.position.y) * 0.5;
        double contactZ = (g_chaser.position.z + g_target.position.z) * 0.5;

        Vec3 contactPoint(0.0, contactY, contactZ);

        double restitution = 0.85;

        double relativeVelocity = g_chaser.velocity.x - g_target.velocity.x;

        double reducedMass =
            (g_chaser.mass * g_target.mass) / (g_chaser.mass + g_target.mass);

        double impulse = (1.0 + restitution) * reducedMass * relativeVelocity;

        double contactForce = impulse / dt;

        Vec3 totalContactForce(contactForce, 0.0, 0.0);

        Vec3 loadCellForces[6];

        DistributeContactForceToLoadCells(totalContactForce, contactPoint, loadCellForces);

        ContactEstimationResult estimate =
            ComputePostContactPose(loadCellForces, g_target.mass, g_target.inertia, dt);

        // Target response
        g_target.velocity += estimate.a_CM * dt;
        g_target.angularVel += estimate.alpha * dt;

        // Chaser response (equal and opposite)
        g_chaser.velocity += estimate.a_CM * (-1.0 * (g_target.mass / g_chaser.mass) * dt);
        g_chaser.angularVel += estimate.alpha * (-0.5 * dt);
    }

    // Move and rotate both cubes
    g_target.position += g_target.velocity * dt;
    g_chaser.position += g_chaser.velocity * dt;

    Quat targetDeltaQuat = Quat::FromAngularDisplacement(g_target.angularVel * dt);
    g_target.attitude = (targetDeltaQuat * g_target.attitude).Normalized();

    Quat chaserDeltaQuat = Quat::FromAngularDisplacement(g_chaser.angularVel * dt);
    g_chaser.attitude = (chaserDeltaQuat * g_chaser.attitude).Normalized();
}


// ============================================================
// 23. LOGGER THREAD
// Background thread that drains the ring buffer into simulation.log (CSV).
// ============================================================
void LoggerThread()
{
    if (!g_config.logEnabled && !g_config.timingLogEnabled)
        return;

    std::ofstream logFile;
    std::ofstream timingFile;

    if (g_config.logEnabled)
    {
        logFile.open("simulation.log");

        if (!logFile.is_open())
        {
            std::cerr << "ERROR: Could not open simulation.log\n";
            g_config.logEnabled = false;
        }
        else
        {
            logFile
                << "timestamp,"
                << "cube1_x,cube1_y,cube1_z,"
                << "cube1_qw,cube1_qx,cube1_qy,cube1_qz,"
                << "cube2_x,cube2_y,cube2_z,"
                << "cube2_qw,cube2_qx,cube2_qy,cube2_qz,"
                << "physics_step\n";

            logFile << std::fixed << std::setprecision(9);
        }
    }

    if (g_config.timingLogEnabled)
    {
        timingFile.open("timestep.csv");

        if (!timingFile.is_open())
        {
            std::cerr << "ERROR: Could not open timestep.csv\n";
            g_config.timingLogEnabled = false;
        }
        else
        {
            // period_us = time since the previous tick (target = physics period)
            // error_us  = period_us - target
            // latency_us = how late the tick started vs. its ideal schedule
            timingFile << "step,timestamp_s,period_us,error_us,latency_us\n";
        }
    }

    constexpr int FLUSH_EVERY_N_ROWS = 2000;
    int rowsSinceFlush = 0;
    const double targetUs = static_cast<double>(g_config.physicsPeriodUs);

    LogRecord record;
    TimingRecord trec;
    char line[128];

    auto drain = [&]() -> bool
    {
        bool any = false;

        while (g_logRing.Pop(record))
        {
            any = true;

            logFile
                << record.timestamp << ","

                << record.cube1Position.x << ","
                << record.cube1Position.y << ","
                << record.cube1Position.z << ","

                << record.cube1Orientation.w << ","
                << record.cube1Orientation.x << ","
                << record.cube1Orientation.y << ","
                << record.cube1Orientation.z << ","

                << record.cube2Position.x << ","
                << record.cube2Position.y << ","
                << record.cube2Position.z << ","

                << record.cube2Orientation.w << ","
                << record.cube2Orientation.x << ","
                << record.cube2Orientation.y << ","
                << record.cube2Orientation.z << ","

                << record.physicsStep
                << "\n";

            if (++rowsSinceFlush >= FLUSH_EVERY_N_ROWS)
            {
                logFile.flush();
                rowsSinceFlush = 0;
            }
        }

        while (g_timingRing.Pop(trec))
        {
            any = true;

            const double periodUs = static_cast<double>(trec.periodNs) / 1000.0;
            const double errorUs  = trec.periodNs == 0 ? 0.0 : periodUs - targetUs;

            const int n = std::snprintf(line, sizeof(line), "%llu,%.9f,%.3f,%.3f,%.3f\n",
                static_cast<unsigned long long>(trec.step),
                static_cast<double>(trec.timestampNs) * 1e-9,
                trec.periodNs == 0 ? targetUs : periodUs,
                errorUs,
                static_cast<double>(trec.latencyNs) / 1000.0);

            timingFile.write(line, n);
        }

        return any;
    };

    for (;;)
    {
        if (!drain())
        {
            if (g_loggerStop.load(std::memory_order_acquire))
            {
                drain();   // physics has stopped: pick up anything pushed last
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    if (logFile.is_open())    { logFile.flush();    logFile.close(); }
    if (timingFile.is_open()) { timingFile.flush(); timingFile.close(); }
}


// ============================================================
// 24. PHYSICS THREAD
// Runs the simulation at a fixed step (default 1000 us): commands, physics,
// publish state, log, then wait for the next tick. Ticks are scheduled on an
// absolute timeline so jitter does not accumulate.
// ============================================================
using PhysClock = std::chrono::steady_clock;

static inline void CpuRelax()
{
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
    _mm_pause();
#elif defined(__aarch64__)
    __asm__ __volatile__("yield");
#else
    std::this_thread::yield();
#endif
}

// How long before the deadline we stop sleeping and start spinning.
// Linux: using clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME) with a 500 us margin
// guarantees the thread wakes up 150-350 us BEFORE the deadline even on standard kernels,
// and spins into the target with nanosecond accuracy, while using ~50% CPU so Linux
// real-time bandwidth throttling (95% default) is never triggered.
// Windows: even with timeBeginPeriod(1) Sleep() is ~1 ms coarse.
#if defined(_WIN32)
constexpr auto SLEEP_MARGIN = std::chrono::microseconds(1500);
#else
inline std::chrono::microseconds GetSleepMargin()
{
    const int marginUs = std::min(500, std::max(100, g_config.physicsPeriodUs / 2));
    return std::chrono::microseconds(marginUs);
}
#endif

static void WaitUntil(PhysClock::time_point target)
{
#if defined(_WIN32)
    constexpr auto sleepMargin = SLEEP_MARGIN;
#else
    const auto sleepMargin = GetSleepMargin();
#endif

    for (;;)
    {
        const auto now = PhysClock::now();

        if (now >= target)
            return;

        const auto remaining = target - now;

        if (!g_config.spin && remaining > sleepMargin)
        {
#if defined(__linux__)
            const auto wakeTime = target - sleepMargin;
            const auto d = wakeTime.time_since_epoch();
            const auto sec = std::chrono::duration_cast<std::chrono::seconds>(d);
            const auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(d - sec);
            struct timespec req;
            req.tv_sec = static_cast<time_t>(sec.count());
            req.tv_nsec = static_cast<long>(nsec.count());
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &req, nullptr);
#else
            std::this_thread::sleep_for(remaining - sleepMargin);
#endif
        }
        else
        {
            // Poll the clock a few times per pause so we notice the deadline
            // within tens of nanoseconds instead of after a full pause.
            for (int i = 0; i < 4; ++i)
            {
                if (PhysClock::now() >= target)
                    return;
            }
            CpuRelax();
        }
    }
}

static int OnlineCpus()
{
#if defined(_WIN32)
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    return static_cast<int>(si.dwNumberOfProcessors);
#elif defined(__linux__)
    const long n = sysconf(_SC_NPROCESSORS_ONLN);
    return n > 0 ? static_cast<int>(n) : 1;
#else
    return 1;
#endif
}

static void ConfigurePhysicsThread()
{
    // Touch the stack once so it is resident before the first tick
    {
        volatile unsigned char stack[128 * 1024];
        for (size_t i = 0; i < sizeof(stack); i += 4096)
            stack[i] = 0;
    }

#if defined(_WIN32)
    timeBeginPeriod(1);

    if (g_config.realtime)
        SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);

    if (g_config.physicsCpu >= 0)
        SetThreadAffinityMask(GetCurrentThread(), static_cast<DWORD_PTR>(1) << g_config.physicsCpu);

#elif defined(__linux__)
    // Remove the default 50 us timer slack so sleeps wake up on time
    prctl(PR_SET_TIMERSLACK, 1UL, 0UL, 0UL, 0UL);
    prctl(PR_SET_NAME, "physics", 0UL, 0UL, 0UL);

    if (g_config.physicsCpu >= 0)
    {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(g_config.physicsCpu, &set);

        const int rc = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);

        if (rc != 0)
            std::cerr << "[WARN] Could not pin physics thread to CPU " << g_config.physicsCpu
                      << " (" << std::strerror(rc) << ")\n";
    }

    if (g_config.realtime)
    {
        sched_param sp{};
        sp.sched_priority = 80;

        int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
        if (rc != 0)
        {
            sp.sched_priority = 50;
            rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
        }

        if (rc != 0)
        {
            std::cerr
                << "[INFO] Real-time priority not available (" << std::strerror(rc) << "); "
                << "running at normal priority.\n"
                << "       For the tightest timing on Ubuntu, either run with sudo or add to\n"
                << "       /etc/security/limits.conf:  @yourgroup - rtprio 50\n";

            // Best effort: at least win against normal-priority threads
            if (setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), -20) != 0)
                std::cerr << "[INFO] Could not raise nice level either; expect jitter from other processes.\n";
        }
    }
#endif
}

static void RestorePhysicsThread()
{
#if defined(_WIN32)
    timeEndPeriod(1);
#endif
}

void PhysicsThread()
{
    ConfigurePhysicsThread();

    using Clock = PhysClock;
    using std::chrono::duration_cast;
    using std::chrono::nanoseconds;

    const auto    period   = std::chrono::microseconds(g_config.physicsPeriodUs);
    const int64_t periodNs = static_cast<int64_t>(g_config.physicsPeriodUs) * 1000;
    const double  dt       = g_config.physicsPeriodUs * 1e-6;
    const int     logEvery = std::max(1, g_config.logEvery);

    const uint64_t ticksPerWindow =
        std::max<uint64_t>(1, 1000000u / static_cast<uint64_t>(g_config.physicsPeriodUs));

    const auto startTime = Clock::now();
    auto nextTick = startTime;

    Clock::time_point prevStart{};
    bool havePrev = false;
    int64_t tickPeriodNs = 0;

    uint64_t physicsStep = 0;

    uint64_t winCount = 0;
    uint64_t winSumNs = 0;
    uint32_t winMaxPeriodNs = 0;
    uint32_t winMaxWorkNs = 0;
    uint32_t winMaxLatencyNs = 0;
    const int64_t tolNs = static_cast<int64_t>(g_config.tolUs * 1000.0);
    const auto    warmup = std::chrono::milliseconds(g_config.warmupMs);

    while (g_running.load(std::memory_order_relaxed))
    {
        const auto tickStart = Clock::now();

        // Scheduling delay: how late this tick started versus its ideal time
        const int64_t latencyNs =
            std::max<int64_t>(0, duration_cast<nanoseconds>(tickStart - nextTick).count());

        // Thread start-up, first-touch page faults and the GL driver spinning up
        // happen in the first moments; they are not steady-state behaviour.
        const bool measuring = (tickStart - startTime) >= warmup;

        {
            const uint32_t l = static_cast<uint32_t>(std::min<int64_t>(latencyNs, UINT32_MAX));
            winMaxLatencyNs = std::max(winMaxLatencyNs, l);

            if (measuring)
            {
                g_timing.runMaxLatencyNs = std::max(g_timing.runMaxLatencyNs, l);

                if (latencyNs > tolNs)
                    g_timing.latencyViolations.fetch_add(1, std::memory_order_relaxed);
            }
        }

        // Period between the starts of consecutive ticks
        tickPeriodNs = 0;

        if (havePrev)
        {
            const int64_t p = duration_cast<nanoseconds>(tickStart - prevStart).count();
            tickPeriodNs = p;
            const uint32_t pNs = static_cast<uint32_t>(std::min<int64_t>(p, UINT32_MAX));

            winSumNs += pNs;
            ++winCount;
            winMaxPeriodNs = std::max(winMaxPeriodNs, pNs);

            if (measuring)
            {
                g_timing.runSumPeriodNs += pNs;
                ++g_timing.runSamples;
                g_timing.runMaxPeriodNs = std::max(g_timing.runMaxPeriodNs, pNs);

                const size_t bucket = std::min<size_t>(pNs / 10000u, PERIOD_HIST_BUCKETS - 1);
                ++g_timing.periodHist[bucket];

                if (p > periodNs + periodNs / 2)
                    g_timing.lateTicks.fetch_add(1, std::memory_order_relaxed);
            }

            if (winCount >= ticksPerWindow)
            {
                g_timing.windowAvgPeriodNs.store(static_cast<uint32_t>(winSumNs / winCount), std::memory_order_relaxed);
                g_timing.windowMaxPeriodNs.store(winMaxPeriodNs, std::memory_order_relaxed);
                g_timing.windowMaxWorkNs.store(winMaxWorkNs, std::memory_order_relaxed);
                g_timing.windowMaxLatencyNs.store(winMaxLatencyNs, std::memory_order_relaxed);
                winCount = 0;
                winSumNs = 0;
                winMaxPeriodNs = 0;
                winMaxWorkNs = 0;
                winMaxLatencyNs = 0;
            }
        }

        prevStart = tickStart;
        havePrev = true;

        // Timestamp log: every tick (also while paused), lock-free push
        if (g_config.timingLogEnabled)
        {
            const TimingRecord tr{
                physicsStep,
                duration_cast<nanoseconds>(tickStart - startTime).count(),
                tickPeriodNs,
                latencyNs
            };

            if (!g_timingRing.Push(tr))
                g_timing.droppedTimingRows.fetch_add(1, std::memory_order_relaxed);
        }

        ProcessCommands();

        UpdatePhysics(dt);

        PublishState(physicsStep);

        // No logging while paused
        if (g_config.logEnabled && !g_paused && (physicsStep % logEvery) == 0)
        {
            LogRecord record;

            // Seconds since the physics thread started
            record.timestamp = std::chrono::duration<double>(tickStart - startTime).count();

            record.cube1Position = g_target.position;
            record.cube1Orientation = g_target.attitude;
            record.cube2Position = g_chaser.position;
            record.cube2Orientation = g_chaser.attitude;
            record.physicsStep = physicsStep;

            if (!g_logRing.Push(record))
                g_timing.droppedLogRows.fetch_add(1, std::memory_order_relaxed);
        }

        physicsStep++;

        const auto workEnd = Clock::now();

        const uint32_t workNs = static_cast<uint32_t>(
            std::min<int64_t>(duration_cast<nanoseconds>(workEnd - tickStart).count(), UINT32_MAX));

        winMaxWorkNs = std::max(winMaxWorkNs, workNs);
        if (measuring)
        {
            g_timing.runMaxWorkNs = std::max(g_timing.runMaxWorkNs, workNs);
            if (workNs >= 1000000u)
                g_timing.workViolations.fetch_add(1, std::memory_order_relaxed);
        }

        nextTick += period;

        // If far behind (e.g. machine was suspended), restart the schedule
        // instead of catching up in a burst
        if (workEnd - nextTick > std::chrono::milliseconds(50))
        {
            nextTick = workEnd;
            havePrev = false;
        }

        WaitUntil(nextTick);
    }

    RestorePhysicsThread();
}



// ============================================================
// 25. RENDER CUBE
// Draws one cube at the given position, orientation and colour.
// ============================================================
void RenderCube(
    GLuint shaderProgram,
    GLuint cubeVAO,
    const Vec3& position,
    const Quat& orientation,
    const Vec3& size,
    const glm::mat4& projection,
    const glm::mat4& view,
    float r, float g, float b)
{
    glm::mat4 model = glm::translate(
        glm::mat4(1.0f),
        glm::vec3(
            static_cast<float>(position.x),
            static_cast<float>(position.y),
            static_cast<float>(position.z)
        )
    );

    model = model * orientation.ToRotationMatrix();

    model = glm::scale(
        model,
        glm::vec3(
            static_cast<float>(size.x),
            static_cast<float>(size.y),
            static_cast<float>(size.z)
        )
    );

    glm::mat4 mvp = projection * view * model;

    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uMVP"), 1, GL_FALSE, glm::value_ptr(mvp));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uModel"), 1, GL_FALSE, glm::value_ptr(model));
    glUniform3f(glGetUniformLocation(shaderProgram, "uColor"), r, g, b);

    glDrawArrays(GL_TRIANGLES, 0, 36);
}


// ============================================================
// 26. ON-SCREEN SIMULATION DATA
// ImGui panel showing live telemetry, cube poses, and simulation controls.
// ============================================================
void SetupImGuiStyle()
{
    ImGuiStyle& style = ImGui::GetStyle();

    style.WindowPadding     = ImVec2(20.0f, 18.0f);
    style.WindowRounding    = 12.0f;
    style.ChildRounding     = 8.0f;
    style.FrameRounding     = 6.0f;
    style.PopupRounding     = 8.0f;
    style.ScrollbarRounding = 8.0f;
    style.GrabRounding      = 4.0f;
    style.TabRounding       = 6.0f;

    style.FramePadding      = ImVec2(10.0f, 6.0f);
    style.ItemSpacing       = ImVec2(10.0f, 8.0f);
    style.ItemInnerSpacing  = ImVec2(8.0f, 5.0f);
    style.WindowBorderSize  = 1.0f;
    style.FrameBorderSize   = 0.0f;

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_Text]                  = ImVec4(0.93f, 0.95f, 0.98f, 1.00f);
    colors[ImGuiCol_TextDisabled]          = ImVec4(0.55f, 0.62f, 0.72f, 1.00f);
    colors[ImGuiCol_WindowBg]              = ImVec4(0.08f, 0.10f, 0.14f, 0.88f);
    colors[ImGuiCol_ChildBg]               = ImVec4(0.11f, 0.14f, 0.20f, 0.50f);
    colors[ImGuiCol_PopupBg]               = ImVec4(0.10f, 0.12f, 0.17f, 0.95f);
    colors[ImGuiCol_Border]                = ImVec4(0.24f, 0.32f, 0.46f, 0.65f);
    colors[ImGuiCol_BorderShadow]          = ImVec4(0.00f, 0.00f, 0.00f, 0.40f);
    colors[ImGuiCol_FrameBg]               = ImVec4(0.14f, 0.18f, 0.26f, 0.70f);
    colors[ImGuiCol_FrameBgHovered]        = ImVec4(0.20f, 0.26f, 0.38f, 0.85f);
    colors[ImGuiCol_FrameBgActive]         = ImVec4(0.25f, 0.32f, 0.46f, 0.95f);
    colors[ImGuiCol_TitleBg]               = ImVec4(0.10f, 0.13f, 0.19f, 1.00f);
    colors[ImGuiCol_TitleBgActive]         = ImVec4(0.14f, 0.19f, 0.28f, 1.00f);
    colors[ImGuiCol_TitleBgCollapsed]      = ImVec4(0.08f, 0.10f, 0.14f, 0.75f);
    colors[ImGuiCol_MenuBarBg]             = ImVec4(0.12f, 0.15f, 0.22f, 1.00f);
    colors[ImGuiCol_ScrollbarBg]           = ImVec4(0.08f, 0.10f, 0.14f, 0.40f);
    colors[ImGuiCol_ScrollbarGrab]         = ImVec4(0.24f, 0.32f, 0.45f, 0.80f);
    colors[ImGuiCol_ScrollbarGrabHovered]  = ImVec4(0.32f, 0.42f, 0.58f, 0.90f);
    colors[ImGuiCol_ScrollbarGrabActive]   = ImVec4(0.40f, 0.52f, 0.72f, 1.00f);
    colors[ImGuiCol_CheckMark]             = ImVec4(0.30f, 0.75f, 0.95f, 1.00f);
    colors[ImGuiCol_SliderGrab]            = ImVec4(0.30f, 0.75f, 0.95f, 0.80f);
    colors[ImGuiCol_SliderGrabActive]      = ImVec4(0.40f, 0.85f, 1.00f, 1.00f);
    colors[ImGuiCol_Button]                = ImVec4(0.16f, 0.22f, 0.32f, 0.85f);
    colors[ImGuiCol_ButtonHovered]         = ImVec4(0.24f, 0.34f, 0.48f, 0.95f);
    colors[ImGuiCol_ButtonActive]          = ImVec4(0.30f, 0.42f, 0.60f, 1.00f);
    colors[ImGuiCol_Header]                = ImVec4(0.18f, 0.25f, 0.36f, 0.70f);
    colors[ImGuiCol_HeaderHovered]         = ImVec4(0.25f, 0.35f, 0.50f, 0.80f);
    colors[ImGuiCol_HeaderActive]          = ImVec4(0.30f, 0.42f, 0.60f, 0.90f);
    colors[ImGuiCol_Separator]             = ImVec4(0.22f, 0.30f, 0.42f, 0.70f);
    colors[ImGuiCol_SeparatorHovered]      = ImVec4(0.30f, 0.42f, 0.60f, 0.80f);
    colors[ImGuiCol_SeparatorActive]       = ImVec4(0.40f, 0.55f, 0.80f, 1.00f);
    colors[ImGuiCol_PlotLines]             = ImVec4(0.40f, 0.75f, 0.95f, 1.00f);
    colors[ImGuiCol_PlotHistogram]         = ImVec4(0.30f, 0.75f, 0.95f, 1.00f);
}

static void DrawCubeBlock(const char* title, const Vec3& p, const Quat& q)
{
    ImGui::Text("%s", title);
    ImGui::Separator();

    ImGui::Text("Position");
    ImGui::Text("  X: % .6f", p.x);
    ImGui::Text("  Y: % .6f", p.y);
    ImGui::Text("  Z: % .6f", p.z);

    ImGui::Spacing();

    ImGui::Text("Quaternion");
    ImGui::Text("  W: % .6f", q.w);
    ImGui::Text("  X: % .6f", q.x);
    ImGui::Text("  Y: % .6f", q.y);
    ImGui::Text("  Z: % .6f", q.z);
}


void RenderSimulationText(const SimulationState& state)
{
    float displayWidth = ImGui::GetIO().DisplaySize.x;
    float displayHeight = ImGui::GetIO().DisplaySize.y;

    float panelWidth = std::clamp(displayWidth * 0.35f, 520.0f, 720.0f);
    float panelHeight = displayHeight - 24.0f;

    ImGui::SetNextWindowPos(ImVec2(12.0f, 12.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(panelWidth, panelHeight), ImGuiCond_Always);

    ImGui::Begin(
        "Simulation State",
        nullptr,
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoCollapse
    );

    DrawCubeBlock("CUBE 1 - TARGET", state.cube1Position, state.cube1Orientation);

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    DrawCubeBlock("CUBE 2 - CHASER", state.cube2Position, state.cube2Orientation);

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::Text("Physics");
    ImGui::Separator();
    ImGui::Text("  Step: %llu", static_cast<unsigned long long>(state.physicsStep));
    {
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - g_guiStartTime).count();
        if (g_config.secondsSpecified)
        {
            const double remain = std::max(0.0, g_config.headlessSeconds - elapsed);
            ImGui::Text("  Timer: %.1f / %.0f s (%.1f s left)", elapsed, g_config.headlessSeconds, remain);
        }
        else
        {
            ImGui::Text("  Elapsed: %.1f s", elapsed);
        }
    }
    ImGui::Text("  Frequency: %.0f Hz", 1e6 / g_config.physicsPeriodUs);
    ImGui::Text("  Timestep: %d us", g_config.physicsPeriodUs);
    ImGui::Text("  Avg period: %.1f us", g_timing.windowAvgPeriodNs.load(std::memory_order_relaxed) / 1000.0);
    ImGui::Text("  Max period: %.1f us", g_timing.windowMaxPeriodNs.load(std::memory_order_relaxed) / 1000.0);
    ImGui::Text("  Max work: %.1f us (< 1 ms)", g_timing.windowMaxWorkNs.load(std::memory_order_relaxed) / 1000.0);
    ImGui::Text("  Max start latency: %.1f us", g_timing.windowMaxLatencyNs.load(std::memory_order_relaxed) / 1000.0);
    ImGui::Text("  Ticks > %.0f us late: %llu", g_config.tolUs, static_cast<unsigned long long>(g_timing.latencyViolations.load(std::memory_order_relaxed)));
    ImGui::Text("  Late ticks: %llu", static_cast<unsigned long long>(g_timing.lateTicks.load(std::memory_order_relaxed)));
    ImGui::Text("  Rendering: ~60 FPS");

    ImGui::End();
}


// ============================================================
// 26b. START-UP HELPERS
// Command-line parsing, state initialisation, clean shutdown, timing report
// and the window-less benchmark mode.
// ============================================================
static void PrintUsage(const char* exe)
{
    std::cout
        << "Usage: " << exe << " [options]\n"
        << "  --dt-us N        physics period in microseconds (default " << DEFAULT_PHYSICS_PERIOD_US
        << ", range " << MIN_PHYSICS_PERIOD_US << "-" << MAX_PHYSICS_PERIOD_US << ")\n"
        << "  --scenario N     1 = head-on (default), 2 = off-center\n"
        << "  --headless       run physics only, no window (benchmark / SSH / CI)\n"
        << "  --seconds S      run time in headless mode (default 10)\n"
        << "  --no-log         do not write simulation.log\n"
        << "  --no-timing-log  do not write timestep.csv (per-tick timestamps)\n"
        << "  --cpu N          pin the physics thread to core N (use an isolated core)\n"
        << "  --spin / --no-spin  force busy-waiting on/off (default: auto, on only if RT throttling is off)\n"
        << "  --no-pin         do not auto-pin the physics thread / auto-enable spin\n"
        << "  --warmup-ms N    ignore the first N ms in the run statistics (default 300)\n"
        << "  --tol-us N       start-delay limit counted as a violation (default 50)\n"
        << "  --log-every N    log every Nth physics step (default 1)\n"
        << "  --no-realtime    do not try to raise the physics thread priority\n"
        << "  -h, --help       show this help\n";
}

// Returns -1 to continue running, otherwise the process exit code.
static int ParseArgs(int argc, char** argv)
{
    auto needValue = [&](int& i, const char* name) -> const char*
    {
        if (i + 1 >= argc)
        {
            std::cerr << "Missing value for " << name << "\n";
            return nullptr;
        }
        return argv[++i];
    };

    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        const char* v = nullptr;

        if (a == "-h" || a == "--help")
        {
            PrintUsage(argv[0]);
            return 0;
        }
        else if (a == "--headless")    g_config.headless = true;
        else if (a == "--no-log")      g_config.logEnabled = false;
        else if (a == "--no-timing-log") g_config.timingLogEnabled = false;
        else if (a == "--spin")      { g_config.spin = true;  g_config.spinForced = true; }
        else if (a == "--no-spin")   { g_config.spin = false; g_config.spinForced = true; }
        else if (a == "--no-pin")      g_config.autoTune = false;
        else if (a == "--warmup-ms")
        {
            if (!(v = needValue(i, "--warmup-ms"))) return 1;
            g_config.warmupMs = std::atoi(v);
            if (g_config.warmupMs < 0) { std::cerr << "--warmup-ms must be >= 0\n"; return 1; }
        }
        else if (a == "--cpu")
        {
            if (!(v = needValue(i, "--cpu"))) return 1;
            g_config.physicsCpu = std::atoi(v);
            if (g_config.physicsCpu < 0) { std::cerr << "--cpu must be >= 0\n"; return 1; }
        }
        else if (a == "--tol-us")
        {
            if (!(v = needValue(i, "--tol-us"))) return 1;
            g_config.tolUs = std::atof(v);
            if (g_config.tolUs < 0.0) { std::cerr << "--tol-us must be >= 0\n"; return 1; }
        }
        else if (a == "--no-realtime") g_config.realtime = false;
        else if (a == "--dt-us")
        {
            if (!(v = needValue(i, "--dt-us"))) return 1;
            g_config.physicsPeriodUs = std::atoi(v);
            if (g_config.physicsPeriodUs < MIN_PHYSICS_PERIOD_US ||
                g_config.physicsPeriodUs > MAX_PHYSICS_PERIOD_US)
            {
                std::cerr << "--dt-us must be between " << MIN_PHYSICS_PERIOD_US
                          << " and " << MAX_PHYSICS_PERIOD_US << "\n";
                return 1;
            }
        }
        else if (a == "--seconds")
        {
            if (!(v = needValue(i, "--seconds"))) return 1;
            g_config.headlessSeconds = std::atof(v);
            if (g_config.headlessSeconds <= 0.0) { std::cerr << "--seconds must be > 0\n"; return 1; }
            g_config.secondsSpecified = true;
        }
        else if (a == "--log-every")
        {
            if (!(v = needValue(i, "--log-every"))) return 1;
            g_config.logEvery = std::atoi(v);
            if (g_config.logEvery < 1) { std::cerr << "--log-every must be >= 1\n"; return 1; }
        }
        else if (a == "--scenario")
        {
            if (!(v = needValue(i, "--scenario"))) return 1;
            const int sc = std::atoi(v);
            if (sc != 1 && sc != 2) { std::cerr << "--scenario must be 1 or 2\n"; return 1; }
            g_requestedScenario = sc;
        }
        else
        {
            std::cerr << "Unknown option: " << a << "\n";
            PrintUsage(argv[0]);
            return 1;
        }
    }

    return -1;
}

static Scenario RequestedScenario()
{
    return g_requestedScenario.load() == 1 ? Scenario::HeadOn : Scenario::OffCenter;
}

static void InitStateBuffers()
{
    for (int i = 0; i < 3; ++i)
    {
        g_stateBuffers[i].state.cube1Position = g_target.position;
        g_stateBuffers[i].state.cube1Orientation = g_target.attitude;
        g_stateBuffers[i].state.cube1Velocity = g_target.velocity;
        g_stateBuffers[i].state.cube2Position = g_chaser.position;
        g_stateBuffers[i].state.cube2Orientation = g_chaser.attitude;
        g_stateBuffers[i].state.cube2Velocity = g_chaser.velocity;
        g_stateBuffers[i].state.physicsStep = 0;
        g_stateBuffers[i].state.contactOccurred = false;
        g_stateBuffers[i].state.paused = false;
        g_stateBuffers[i].state.scenario = static_cast<int>(g_currentScenario);
        g_stateBuffers[i].sequence.store(0);
    }
}

// Stop physics first, then let the logger finish writing what is left.
static void StopSimThreads(std::thread& physicsThread, std::thread& loggerThread)
{
    g_running = false;

    if (physicsThread.joinable())
        physicsThread.join();

    g_loggerStop.store(true, std::memory_order_release);

    if (loggerThread.joinable())
        loggerThread.join();
}

static void PrintTimingSummary()
{
    const uint64_t n = g_timing.runSamples;

    if (n == 0)
        return;

    // Upper edge (in us) of the histogram bucket that holds the given percentile
    auto percentileUs = [&](double p) -> int
    {
        const uint64_t target = static_cast<uint64_t>(std::ceil(p * static_cast<double>(n)));
        uint64_t cumulative = 0;

        for (int b = 0; b < PERIOD_HIST_BUCKETS; ++b)
        {
            cumulative += g_timing.periodHist[b];
            if (cumulative >= target)
                return (b + 1) * 10;
        }

        return PERIOD_HIST_BUCKETS * 10;
    };

    const double avgUs = static_cast<double>(g_timing.runSumPeriodNs) / static_cast<double>(n) / 1000.0;

    std::printf("\n---------------- physics timing ----------------\n");
    std::printf("target period   : %d us (%.0f Hz)\n", g_config.physicsPeriodUs, 1e6 / g_config.physicsPeriodUs);
    std::printf("ticks measured  : %llu\n", static_cast<unsigned long long>(n));
    std::printf("average period  : %.2f us\n", avgUs);
    std::printf("p99 / p99.9     : <= %d us / <= %d us  (10 us buckets)\n", percentileUs(0.99), percentileUs(0.999));
    std::printf("max period      : %.1f us\n", g_timing.runMaxPeriodNs / 1000.0);
    std::printf("max start delay : %.1f us  (tick start vs. its schedule)\n", g_timing.runMaxLatencyNs / 1000.0);
    std::printf("ticks > %.0f us late: %llu\n", g_config.tolUs,
                static_cast<unsigned long long>(g_timing.latencyViolations.load()));
    std::printf("max work / tick : %.2f us  (physics+publish+log push%s)\n",
                g_timing.runMaxWorkNs / 1000.0,
                g_timing.runMaxWorkNs < 1000000u ? " - STRICTLY < 1 ms" : " - EXCEEDED 1 ms");
    if (g_timing.workViolations.load() > 0)
        std::printf("work > 1 ms     : %llu violations!\n",
                    static_cast<unsigned long long>(g_timing.workViolations.load()));
    std::printf("late ticks      : %llu  (period > 1.5x target)\n",
                static_cast<unsigned long long>(g_timing.lateTicks.load()));
    std::printf("dropped log rows: %llu\n", static_cast<unsigned long long>(g_timing.droppedLogRows.load()));
    std::printf("dropped timing rows: %llu\n", static_cast<unsigned long long>(g_timing.droppedTimingRows.load()));
    if (g_config.timingLogEnabled)
        std::printf("per-tick timestamps: timestep.csv  (check with: python3 check_timing.py)\n");
    std::printf("------------------------------------------------\n");
}

#if defined(__linux__)
static std::vector<int> GetCpuSiblings(int cpu)
{
    std::vector<int> siblings;
    siblings.push_back(cpu);

    const std::string path = "/sys/devices/system/cpu/cpu" + std::to_string(cpu) + "/topology/thread_siblings_list";
    std::ifstream file(path);
    if (!file.is_open())
        return siblings;

    std::string line;
    if (std::getline(file, line))
    {
        siblings.clear();
        std::stringstream ss(line);
        std::string token;
        while (std::getline(ss, token, ','))
        {
            const auto dash = token.find('-');
            if (dash != std::string::npos)
            {
                const int start = std::stoi(token.substr(0, dash));
                const int end = std::stoi(token.substr(dash + 1));
                for (int c = start; c <= end; ++c)
                    siblings.push_back(c);
            }
            else if (!token.empty())
            {
                siblings.push_back(std::stoi(token));
            }
        }
    }
    if (siblings.empty())
        siblings.push_back(cpu);
    return siblings;
}
#endif

// Called once from main, before any thread is started.
static void PrepareProcessForRealtime()
{
    const int ncpu = OnlineCpus();

    // With enough cores, dedicate the last one to physics and busy-wait on it.
    // Sleeping is the biggest jitter source: a wake-up that overshoots by even
    // 200 us makes that tick late.
    if (g_config.autoTune && g_config.realtime && ncpu >= 3)
    {
        if (g_config.physicsCpu < 0)
            g_config.physicsCpu = ncpu - 1;

        // Busy-waiting uses 100% of the core. Under SCHED_FIFO the kernel then
        // throttles the thread for ~50 ms every second (sched_rt_runtime_us =
        // 950000 is the Ubuntu default), which is far worse than any sleep
        // jitter. So only auto-enable spin when throttling is disabled.
        if (!g_config.spinForced)
        {
            long rtRuntime = 950000;
#if defined(__linux__)
            if (FILE* f = std::fopen("/proc/sys/kernel/sched_rt_runtime_us", "r"))
            {
                if (std::fscanf(f, "%ld", &rtRuntime) != 1)
                    rtRuntime = 950000;
                std::fclose(f);
            }
#endif
            g_config.spin = (rtRuntime == -1);
        }
    }

    if (g_config.physicsCpu >= ncpu)
    {
        std::cerr << "[WARN] --cpu " << g_config.physicsCpu << " does not exist (" << ncpu
                  << " cores); not pinning.\n";
        g_config.physicsCpu = -1;
    }

    if (g_config.spin && ncpu < 2)
    {
        // A spinning real-time thread on the only core would starve everything else
        std::cerr << "[WARN] --spin needs at least 2 CPU cores; using sleep+spin instead.\n";
        g_config.spin = false;
    }

    // Avoid first-touch page faults inside the physics loop
    g_logRing.Prefault();
    g_timingRing.Prefault();

    if (!g_config.realtime)
        return;

#if defined(_WIN32)
    if (!SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS))
        SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);

#elif defined(__linux__)
    // Keep everything other than the physics core AND its hardware SMT siblings for the main / render / logger threads
    if (g_config.physicsCpu >= 0 && ncpu > 1)
    {
        const std::vector<int> siblings = GetCpuSiblings(g_config.physicsCpu);

        cpu_set_t others;
        CPU_ZERO(&others);

        int otherCount = 0;
        for (int c = 0; c < ncpu; ++c)
        {
            if (std::find(siblings.begin(), siblings.end(), c) == siblings.end())
            {
                CPU_SET(c, &others);
                otherCount++;
            }
        }

        if (otherCount > 0)
        {
            sched_setaffinity(0, sizeof(others), &others);
            std::cout << "[INFO] Physics pinned to CPU " << g_config.physicsCpu
                      << ". Physical core isolated: SMT siblings (";
            for (size_t i = 0; i < siblings.size(); ++i)
                std::cout << (i > 0 ? ", " : "") << siblings[i];
            std::cout << ") excluded from GUI/render/logger threads.\n";
        }
        else
        {
            for (int c = 0; c < ncpu; ++c)
                if (c != g_config.physicsCpu)
                    CPU_SET(c, &others);
            sched_setaffinity(0, sizeof(others), &others);
        }
    }

    // Keep pages resident (only when the limit allows it; a small limit would
    // make later allocations fail)
    rlimit lim{};
    // MCL_ONFAULT locks pages as they are touched, so a modest memlock limit is
    // usually enough. A page fault inside the physics loop costs far more than 1 ms
    // when memory is under pressure.
    (void)lim;
    getrlimit(RLIMIT_MEMLOCK, &lim);
#ifndef MCL_ONFAULT
#define MCL_ONFAULT 4
#endif
    if (mlockall(MCL_CURRENT | MCL_FUTURE | MCL_ONFAULT) != 0)
        std::cerr << "[INFO] mlockall failed (" << std::strerror(errno)
                  << "); page faults may add jitter. Run with sudo or raise 'ulimit -l'.\n";

    // If permissions allow (e.g. running with sudo), request zero C-state exit latency to prevent sleep-wake jitter
    static int s_cpuDmaFd = -1;
    if (s_cpuDmaFd < 0)
    {
        s_cpuDmaFd = open("/dev/cpu_dma_latency", O_RDWR | O_CLOEXEC);
        if (s_cpuDmaFd >= 0)
        {
            int32_t lat = 0;
            if (write(s_cpuDmaFd, &lat, sizeof(lat)) != sizeof(lat))
            {
                close(s_cpuDmaFd);
                s_cpuDmaFd = -1;
            }
        }
    }

    // The kernel pauses real-time threads for 50 ms every second by default
    // (RT throttling). That is exactly the kind of stall we must avoid.
    if (FILE* f = std::fopen("/proc/sys/kernel/sched_rt_runtime_us", "r"))
    {
        long rtRuntime = 0;
        if (std::fscanf(f, "%ld", &rtRuntime) == 1 && rtRuntime != -1 && g_config.spin)
            std::cerr
                << "[WARN] kernel.sched_rt_runtime_us = " << rtRuntime << ": a spinning real-time thread\n"
                << "       will be throttled (paused) for up to 50 ms every second.\n"
                << "       Run: sudo sysctl -w kernel.sched_rt_runtime_us=-1   (or use run_realtime.sh)\n";
        std::fclose(f);
    }
#endif
}

static int RunHeadless()
{
    SetupScenario(RequestedScenario());
    InitStateBuffers();

    std::thread loggerThread(LoggerThread);
    std::thread physicsThread(PhysicsThread);

    std::this_thread::sleep_for(std::chrono::duration<double>(g_config.headlessSeconds));

    StopSimThreads(physicsThread, loggerThread);
    PrintTimingSummary();

    return 0;
}


// ============================================================
// 27. MAIN
// Sets up the window, graphics and threads, runs the render loop, then shuts down.
// ============================================================
int main(int argc, char** argv)
{
    const int parseResult = ParseArgs(argc, argv);

    if (parseResult >= 0)
        return parseResult;

    PrepareProcessForRealtime();

    std::cout
        << "====================================================\n"
        << " RIGID BODY DOCKING SIMULATION\n"
        << "====================================================\n"
        << "Physics timestep : " << g_config.physicsPeriodUs << " us ("
        << std::fixed << std::setprecision(0) << 1e6 / g_config.physicsPeriodUs << " Hz)\n"
        << std::defaultfloat << std::setprecision(6)
        << "Rendering        : ~60 FPS\n"
        << "Logging          : " << (g_config.logEnabled ? "asynchronous -> simulation.log" : "off") << "\n"
        << "Keys             : 1 / 2 scenario, R reset, SPACE pause\n"
        << "====================================================\n";

    if (g_config.headless)
        return RunHeadless();

    // Window and OpenGL context
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW\n";
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4);

    g_window = glfwCreateWindow(1100, 700, "Rigid Body Docking Simulation", nullptr, nullptr);

    if (!g_window)
    {
        std::cerr << "Failed to create window\n";
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(g_window);

    glfwSwapInterval(1);

    glfwSetKeyCallback(g_window, KeyCallback);

    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
    {
        std::cerr << "Failed to initialize GLAD\n";
        glfwDestroyWindow(g_window);
        glfwTerminate();
        return -1;
    }

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    // Shaders and mesh
    GLuint shaderProgram = CreateShaderProgram();

    if (!shaderProgram)
    {
        std::cerr << "Failed to create shader program\n";
        glfwDestroyWindow(g_window);
        glfwTerminate();
        return -1;
    }

    GLuint cubeVAO;
    GLuint cubeVBO;

    CreateCubeMesh(cubeVAO, cubeVBO);

    // ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    io.FontGlobalScale = 2.0f;

    SetupImGuiStyle();

    ImGui_ImplGlfw_InitForOpenGL(g_window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // Initial scenario and triple buffer
    SetupScenario(RequestedScenario());
    InitStateBuffers();

    // Start threads
    std::thread loggerThread(LoggerThread);
    std::thread physicsThread(PhysicsThread);

    g_guiStartTime = std::chrono::steady_clock::now();

    // Render loop
    while (!glfwWindowShouldClose(g_window))
    {
        if (g_config.secondsSpecified)
        {
            const double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - g_guiStartTime).count();
            if (elapsed >= g_config.headlessSeconds)
            {
                std::cout << "\n[INFO] Reached requested run duration of " << g_config.headlessSeconds << " s. Exiting.\n";
                break;
            }
        }

        if (const int notice = g_pauseNotice.exchange(0))
            std::cout << (notice == 1 ? ">>> PAUSED\n" : ">>> RESUMED\n");

        int windowWidth = 0;
        int windowHeight = 0;

        glfwGetFramebufferSize(g_window, &windowWidth, &windowHeight);

        // Skip drawing while minimised
        if (windowWidth == 0 || windowHeight == 0)
        {
            glfwWaitEvents();
            continue;
        }

        glViewport(0, 0, windowWidth, windowHeight);

        SimulationState renderState = GetLatestRenderState();

        // Dynamically scale font based on resolution (really big and easy to read)
        ImGuiIO& io = ImGui::GetIO();
        if (windowHeight >= 1000)
            io.FontGlobalScale = 2.40f;
        else if (windowHeight >= 850)
            io.FontGlobalScale = 2.10f;
        else
            io.FontGlobalScale = 1.90f;

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        glClearColor(0.12f, 0.14f, 0.18f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUseProgram(shaderProgram);

        // Camera
        float aspectRatio =
            static_cast<float>(windowWidth) / static_cast<float>(windowHeight);

        glm::mat4 projection =
            glm::perspective(glm::radians(42.0f), aspectRatio, 0.1f, 50.0f);

        glm::mat4 view =
            glm::lookAt(
                glm::vec3(0.0f, 3.8f, 6.8f),
                glm::vec3(0.0f, 0.2f, 0.0f),
                glm::vec3(0.0f, 1.0f, 0.0f)
            );

        glBindVertexArray(cubeVAO);

        // Cube 1 (target)
        RenderCube(
            shaderProgram, cubeVAO,
            renderState.cube1Position, renderState.cube1Orientation,
            Vec3(1.0, 1.0, 1.0),
            projection, view,
            0.25f, 0.70f, 0.90f
        );

        // Cube 2 (chaser)
        RenderCube(
            shaderProgram, cubeVAO,
            renderState.cube2Position, renderState.cube2Orientation,
            Vec3(1.0, 1.0, 1.0),
            projection, view,
            0.95f, 0.45f, 0.25f
        );

        // Floor
        glm::mat4 floorModel = glm::mat4(1.0f);
        floorModel = glm::translate(floorModel, glm::vec3(0.0f, -0.65f, 0.0f));
        floorModel = glm::scale(floorModel, glm::vec3(12.0f, 0.02f, 3.0f));

        glm::mat4 floorMVP = projection * view * floorModel;

        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uMVP"), 1, GL_FALSE, glm::value_ptr(floorMVP));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uModel"), 1, GL_FALSE, glm::value_ptr(floorModel));
        glUniform3f(glGetUniformLocation(shaderProgram, "uColor"), 0.18f, 0.20f, 0.25f);

        glDrawArrays(GL_TRIANGLES, 0, 36);

        glBindVertexArray(0);

        // On-screen text
        RenderSimulationText(renderState);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(g_window);
        glfwPollEvents();
    }

    // Shutdown: stop physics first, then let the logger finish writing
    StopSimThreads(physicsThread, loggerThread);

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glDeleteVertexArrays(1, &cubeVAO);
    glDeleteBuffers(1, &cubeVBO);
    glDeleteProgram(shaderProgram);

    glfwDestroyWindow(g_window);
    glfwTerminate();

    PrintTimingSummary();

    std::cout
        << "\nSimulation terminated.\n";

    if (g_config.logEnabled)
        std::cout << "Log written to simulation.log\n";

    return 0;
}
