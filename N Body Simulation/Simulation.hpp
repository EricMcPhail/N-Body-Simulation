#pragma once
#include <vector>
#include <chrono>
#include <mutex>
#include <glm/glm.hpp>

struct GLFWwindow;
class Camera;
class Shader;
class Model;
class ParticleManager;
class ThreadPool;

class Simulation {
public:
    ThreadPool* thread_pool;
    bool is_physics_thread_complete;

    long double run_time = 0.0;
    std::chrono::time_point<std::chrono::high_resolution_clock> tick_start_time_nanoseconds;
    std::chrono::time_point<std::chrono::high_resolution_clock> tick_end_time_nanoseconds;
    //std::chrono::duration<long long, std::nano> tick_delta_time_nanoseconds;
    const size_t target_framerate = 120;


    ParticleManager* pm;
    Model* current_particle_model;
    std::vector<Model> loaded_models;

    // settings
    const unsigned int SCR_WIDTH = 800;
    const unsigned int SCR_HEIGHT = 600;
    GLFWwindow* window;
    Shader* shader;
    Model* model;

    // camera
    Camera* camera;

    float lastX;
    float lastY;
    bool firstMouse = true;




    const double physics_timestep = 0.01;

    //std::mutex m;
    //const std::lock_guard<std::mutex> lock(m);

    Simulation();
    ~Simulation();

	void Init();
    void loadDefaultParticleModel();
	void drawAllParticles(const Model& particle_model) const;
    void updateViewAndProjection();
	void updatePhysics(double delta_time);
    void f(double dt);
    void updateAllParticlesPositionDataInGPU(Model &particle_model, const glm::vec3 &scale = glm::vec3{ 1.0f, 1.0f, 1.0f }) const;
    void Render();
    void startMultiThreaded();
    void startSingleThreaded();

};

static Simulation* CURRENT_CONTROLLED_SIMULATION;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow* window);


