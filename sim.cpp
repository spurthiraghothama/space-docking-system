// Rigid body docking simulation: a moving chaser cube hits a stationary target
// cube. Physics runs at 500 Hz on its own thread, rendering at ~60 FPS, and the
// data is logged to simulation.log. Keys: 1/2 scenario, R reset, SPACE pause.

#include <cmath>
#include <iostream>
#include <vector>
#include <iomanip>
#include <string>
#include <algorithm>

#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <fstream>
#include <queue>
#include <chrono>
#include <cstdint>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <mmsystem.h>
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
struct StateBuffer
{
    SimulationState state;
    std::atomic<uint64_t> sequence{0};
};

StateBuffer g_stateBuffers[3];

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

std::queue<LogRecord> g_logQueue;
std::mutex g_logMutex;
std::condition_variable g_logCV;


// ============================================================
// 16. WINDOW
// ============================================================
GLFWwindow* g_window = nullptr;


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

        std::cout << (g_paused ? ">>> PAUSED\n" : ">>> RESUMED\n");
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
// Background thread that writes queued records to simulation.log (CSV).
// ============================================================
void LoggerThread()
{
    std::ofstream logFile("simulation.log");

    if (!logFile.is_open())
    {
        std::cerr << "ERROR: Could not open simulation.log\n";
        return;
    }

    logFile
        << "timestamp,"
        << "cube1_x,cube1_y,cube1_z,"
        << "cube1_qw,cube1_qx,cube1_qy,cube1_qz,"
        << "cube2_x,cube2_y,cube2_z,"
        << "cube2_qw,cube2_qx,cube2_qy,cube2_qz,"
        << "physics_step\n";

    logFile << std::fixed << std::setprecision(9);

    constexpr int FLUSH_EVERY_N_ROWS = 500;
    int rowsSinceFlush = 0;

    while (true)
    {
        std::vector<LogRecord> batch;

        {
            std::unique_lock<std::mutex> lock(g_logMutex);

            g_logCV.wait(lock, []()
            {
                return !g_logQueue.empty() || g_loggerStop.load();
            });

            while (!g_logQueue.empty())
            {
                batch.push_back(g_logQueue.front());
                g_logQueue.pop();
            }

            if (batch.empty() && g_loggerStop.load())
                break;
        }

        for (const LogRecord& record : batch)
        {
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
    }

    logFile.flush();
    logFile.close();
}


// ============================================================
// 24. PHYSICS THREAD
// Runs the simulation at a fixed 2 ms step (500 Hz): commands, physics,
// publish state, log, then wait for the next tick.
// ============================================================
void PhysicsThread()
{
#ifdef _WIN32
    timeBeginPeriod(1);
#endif

    using Clock = std::chrono::steady_clock;

    constexpr double PHYSICS_DT = 0.002;

    constexpr auto PHYSICS_PERIOD = std::chrono::microseconds(2000);

    const auto startTime = Clock::now();

    auto nextTick = Clock::now();

    uint64_t physicsStep = 0;

    while (g_running)
    {
        ProcessCommands();

        auto physicsStart = Clock::now();

        UpdatePhysics(PHYSICS_DT);

        auto physicsEnd = Clock::now();

        auto physicsDuration =
            std::chrono::duration_cast<std::chrono::microseconds>(
                physicsEnd - physicsStart
            ).count();

        if (physicsDuration > 2000)
        {
            std::cerr << "[PHYSICS OVERRUN] " << physicsDuration << " us\n";
        }

        PublishState(physicsStep);

        // No logging while paused
        if (!g_paused)
        {
            LogRecord record;

            // Seconds since the physics thread started
            record.timestamp =
                std::chrono::duration<double>(Clock::now() - startTime).count();

            record.cube1Position = g_target.position;
            record.cube1Orientation = g_target.attitude;
            record.cube2Position = g_chaser.position;
            record.cube2Orientation = g_chaser.attitude;
            record.physicsStep = physicsStep;

            {
                std::lock_guard<std::mutex> lock(g_logMutex);
                g_logQueue.push(record);
            }

            g_logCV.notify_one();
        }

        physicsStep++;

        nextTick += PHYSICS_PERIOD;

        // If far behind, restart the schedule instead of catching up in a burst
        auto now = Clock::now();
        if (now - nextTick > std::chrono::milliseconds(50))
            nextTick = now;

        // High-precision pacing to hit precisely 2.0 ms (500 Hz):
        // Sleep if more than 1.5 ms remains, then spin-wait the remaining fraction
        while (Clock::now() < nextTick)
        {
            auto remaining = nextTick - Clock::now();
            if (remaining > std::chrono::microseconds(1500))
            {
                std::this_thread::sleep_for(std::chrono::microseconds(500));
            }
        }
    }

#ifdef _WIN32
    timeEndPeriod(1);
#endif
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
    ImGui::Text("  Frequency: 500 Hz");
    ImGui::Text("  Timestep: 2 ms");
    ImGui::Text("  Rendering: ~60 FPS");

    ImGui::End();
}


// ============================================================
// 27. MAIN
// Sets up the window, graphics and threads, runs the render loop, then shuts down.
// ============================================================
int main()
{
    std::cout
        << "====================================================\n"
        << " RIGID BODY DOCKING SIMULATION\n"
        << "====================================================\n"
        << "Physics timestep : 2 ms (500 Hz)\n"
        << "Rendering        : ~60 FPS\n"
        << "Logging          : asynchronous -> simulation.log\n"
        << "Keys             : 1 / 2 scenario, R reset, SPACE pause\n"
        << "====================================================\n";

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
    SetupScenario(Scenario::HeadOn);

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
        g_stateBuffers[i].state.scenario = 1;
        g_stateBuffers[i].sequence.store(0);
    }

    // Start threads
    std::thread loggerThread(LoggerThread);
    std::thread physicsThread(PhysicsThread);

    // Render loop
    while (!glfwWindowShouldClose(g_window))
    {
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
    g_running = false;

    if (physicsThread.joinable())
        physicsThread.join();

    {
        std::lock_guard<std::mutex> lock(g_logMutex);
        g_loggerStop = true;
    }

    g_logCV.notify_all();

    if (loggerThread.joinable())
        loggerThread.join();

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glDeleteVertexArrays(1, &cubeVAO);
    glDeleteBuffers(1, &cubeVBO);
    glDeleteProgram(shaderProgram);

    glfwDestroyWindow(g_window);
    glfwTerminate();

    std::cout
        << "\nSimulation terminated.\n"
        << "Log written to simulation.log\n";

    return 0;
}
