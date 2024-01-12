#include "Simulation.hpp"
#include "Camera.hpp"
#include "Model.hpp"
#include "Shader.hpp"
#include "ParticleManager.hpp"
#include "ThreadPool.hpp"
#include "Particle.hpp"
#include "TextRenderer.hpp"


#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>

#include  <algorithm> // for min
#include <iostream> // for std::cout

#define DRAW_DEBUG_DATA_IN_WINDOW 1
#if DRAW_DEBUG_DATA_IN_WINDOW
#include <format>
#endif


using vec3 = glm::highp_dvec3;

std::vector<glm::vec3> temp_colours = {
    glm::vec3{1.0f, 1.0f,1.0f},
    glm::vec3{1.0f, 0.0f, 0.0f} ,
    glm::vec3{0.0f, 1.0f, 0.0f} ,
    glm::vec3{0.0f, 0.0f, 1.0f} ,
    glm::vec3{1.0f, 1.0f, 0.0f} ,
    glm::vec3{0.0f, 1.0f, 1.0f} ,
    glm::vec3{1.0f, 0.0f, 1.0f} };


// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow* window) {
    if (CURRENT_CONTROLLED_SIMULATION) {
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, true);
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            CURRENT_CONTROLLED_SIMULATION->camera->ProcessKeyboard(FORWARD, CURRENT_CONTROLLED_SIMULATION->physics_timestep);
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            CURRENT_CONTROLLED_SIMULATION->camera->ProcessKeyboard(BACKWARD, CURRENT_CONTROLLED_SIMULATION->physics_timestep);
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            CURRENT_CONTROLLED_SIMULATION->camera->ProcessKeyboard(LEFT, CURRENT_CONTROLLED_SIMULATION->physics_timestep);
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            CURRENT_CONTROLLED_SIMULATION->camera->ProcessKeyboard(RIGHT, CURRENT_CONTROLLED_SIMULATION->physics_timestep);
    }
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xposIn, double yposIn) {
    if (CURRENT_CONTROLLED_SIMULATION) {
        float xpos = static_cast<float>(xposIn);
        float ypos = static_cast<float>(yposIn);

        if (CURRENT_CONTROLLED_SIMULATION->firstMouse)
        {
            CURRENT_CONTROLLED_SIMULATION->lastX = xpos;
            CURRENT_CONTROLLED_SIMULATION->lastY = ypos;
            CURRENT_CONTROLLED_SIMULATION->firstMouse = false;
        }

        float xoffset = xpos - CURRENT_CONTROLLED_SIMULATION->lastX;
        float yoffset = CURRENT_CONTROLLED_SIMULATION->lastY - ypos; // reversed since y-coordinates go from bottom to top

        CURRENT_CONTROLLED_SIMULATION->lastX = xpos;
        CURRENT_CONTROLLED_SIMULATION->lastY = ypos;

        CURRENT_CONTROLLED_SIMULATION->camera->ProcessMouseMovement(xoffset, yoffset);
    }
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    if (CURRENT_CONTROLLED_SIMULATION) {
        CURRENT_CONTROLLED_SIMULATION->camera->ProcessMouseScroll(static_cast<float>(yoffset));
    }
}

Simulation::Simulation() {
    CURRENT_CONTROLLED_SIMULATION = this;
    lastX = (float)SCR_WIDTH / 2.0f;
    lastY = (float)SCR_HEIGHT / 2.0f;
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    // --------------------
    window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "N Body Simulation", NULL, NULL);
    if (window == NULL) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        throw -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cout << "Failed to initialize GLAD" << std::endl;
        throw -1;
    }




    //----------------------------------------------------------------------------
    thread_pool = new ThreadPool();
    shader = new Shader("vertex_shader.vs", "fragment_shader.fs");
    camera = new Camera(glm::vec3(0.0f, 0.0f, -25.0f), glm::vec3(0.0f, 1.0f, 0.0f), 90.0f, 0.0f);
    loadDefaultParticleModel();
}

Simulation::~Simulation() {
    delete shader;
    delete camera;
    delete current_particle_model;
    delete pm;
    delete thread_pool; // TODO MAKE SURE EVERYTHING IS DONE RUNNING
    glfwTerminate();
}

#if 0
void Simulation::Init() {
    ThreeBodyTest f;




    std::vector<Circle*> circles;
    float radius = 1.0f;
    size_t num_tri = 25;
    circles.push_back(new Circle{ radius, num_tri, temp_colours[0] });
    for (size_t i = 0; i < f.integration_methods.size(); i++) {
        circles.push_back(new Circle{ radius, num_tri, temp_colours[f.integration_methods[i] + 1] });
    }





    //std::vector<glm::mat4> m(f.exact_solution.particles.size());
    ParticleManager pm;

    pm.add(1.0, vec3{ -2.0, 0.0, 0.0 });
    pm.add(1.0, vec3{ 2.0, 0.0, 0.0 });

    CollisionEventManager cem = CollisionEventManager{ &pm };
    double TE_init = cem.pm->getTotalEnergy();
}
#endif

void Simulation::updateViewAndProjection() {
    // This should be run every time we want to update the frame
    // configure transformation matrices
    glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 1000.0f);
    glm::mat4 view = camera->GetViewMatrix();
    shader->use();
    shader->setMat4("projection", projection);
    shader->setMat4("view", view);
}

void Simulation::updatePhysics(double delta_time) {
    pm->updateAndResolveCollisions(delta_time, 1);
}

#if 0
void Simulation::startMultiThreaded() {
    tick_end_time_nanoseconds = std::chrono::high_resolution_clock::now();

    const size_t target_framerate = 120;
    const double number_frames_between_physics_updates = target_framerate * physics_timestep;
    const double target_frametime = 1.0 / target_framerate;
    const double target_physicstime = 1.0 / number_frames_between_physics_updates;


    thread_pool->Start();
    thread_pool->QueueJob(std::bind(updatePhysics, *this, physics_timestep));

    ctpl::thread_pool p(21);
    std::vector<std::future<void>> results(4);
    results[2] = p.push(std::bind(updatePhysics, this, physics_timestep));

 
    auto x = [this] { updatePhysics(physics_timestep); };
    std::future<void> physics_update(x);



    while (!glfwWindowShouldClose(window)) {
        // per-frame time logic
        // --------------------
        tick_start_time_nanoseconds = std::chrono::high_resolution_clock::now();
        const std::chrono::duration<long long, std::nano> tick_delta_time_nanoseconds = tick_start_time_nanoseconds - tick_end_time_nanoseconds;
        tick_end_time_nanoseconds = tick_start_time_nanoseconds;
        // Rest of the simulation loop
        const std::chrono::duration<long double> tick_delta_time_seconds = tick_delta_time_nanoseconds;
        const double delta_time = tick_delta_time_seconds.count();

        // input
        // -----
        processInput(window);

        // physics
        // -------
        updatePhysics(physics_timestep);
        updateAllParticlesPositionDataInGPU(*current_particle_model);

        // render
        // ------
        updateViewAndProjection();
        drawAllParticles(*current_particle_model);

        // ending stuff
        // ------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    thread_pool->Stop();
}
#endif

void Simulation::startSingleThreaded() {
    pm->add(std::numeric_limits<double>::infinity(), Vector{ -5.0, 0.0,0.0 });
    pm->add(1.0, Vector{5.0,0.0,0.0});
    pm->add(1000000.0, Vector{ 15.0,0.0,0.0 }, Vector{-1.0, 0.0, 0.0});
    

#if DRAW_DEBUG_DATA_IN_WINDOW
    TextRenderer tr(SCR_WIDTH, SCR_HEIGHT);
#endif 

    pm->particles[0].is_affected_by_gravity = false;
    pm->particles[1].is_affected_by_gravity = false;
    pm->particles[2].is_affected_by_gravity = false;


    // Time Debugging Variables

    auto input_start_time_nanoseconds = std::chrono::high_resolution_clock::now();
    auto input_end_time_nanoseconds = std::chrono::high_resolution_clock::now();

    auto physics_start_time_nanoseconds = std::chrono::high_resolution_clock::now();
    auto physics_end_time_nanoseconds = std::chrono::high_resolution_clock::now();

    auto gpu_data_update_start_time_nanoseconds = std::chrono::high_resolution_clock::now();
    auto gpu_data_update_end_time_nanoseconds = std::chrono::high_resolution_clock::now();

    auto render_start_time_nanoseconds = std::chrono::high_resolution_clock::now();
    auto render_end_time_nanoseconds = std::chrono::high_resolution_clock::now();
    
    long double input_delta_time_seconds_sum = 0.0;
    long double physics_delta_time_seconds_sum = 0.0;
    long double gpu_data_update_delta_time_seconds_sum = 0.0;
    long double render_delta_time_seconds_sum = 0.0;
    long double everything_else_delta_time_seconds_sum = 0.0;

    long double input_delta_time_seconds_average = 0.0;
    long double physics_delta_time_seconds_average = 0.0;
    long double gpu_data_update_delta_time_seconds_average = 0.0;
    long double render_delta_time_seconds_average = 0.0;
    long double everything_else_delta_time_seconds_average = 0.0;


    // FPS Variables
    long double FPS = 0.0;
    unsigned int number_of_frames = 0;
    long double time_between_last_FPS_calculation = 0.0;
    const long double time_between_last_FPS_calculation_target = 3.0;
    // Rendering and Physics Targets
    const double target_frametime = 1.0 / target_framerate;
    const double number_frames_between_physics_updates = target_framerate * physics_timestep;
    const double target_physicstime = 1.0 / number_frames_between_physics_updates;
    // Start of the actual game loop
    tick_end_time_nanoseconds = std::chrono::high_resolution_clock::now();
    while (!glfwWindowShouldClose(window)) {
        // per-frame time logic
        // --------------------
        tick_start_time_nanoseconds = std::chrono::high_resolution_clock::now();
        const std::chrono::duration<long long, std::nano> tick_delta_time_nanoseconds = tick_start_time_nanoseconds - tick_end_time_nanoseconds;
        tick_end_time_nanoseconds = tick_start_time_nanoseconds;
        // FPS Calculation
        // ---------------
        const std::chrono::duration<long double> tick_delta_time_seconds = tick_delta_time_nanoseconds;
        const long double delta_time = tick_delta_time_seconds.count();
        time_between_last_FPS_calculation += delta_time;

        // Debug Time Calculations
        // -----------------------
        const std::chrono::duration<long long, std::nano> input_delta_time_nanoseconds =  input_end_time_nanoseconds - input_start_time_nanoseconds;
        const std::chrono::duration<long long, std::nano> physics_delta_time_nanoseconds =  physics_end_time_nanoseconds - physics_start_time_nanoseconds;
        const std::chrono::duration<long long, std::nano> gpu_data_update_delta_time_nanoseconds =  gpu_data_update_end_time_nanoseconds - gpu_data_update_start_time_nanoseconds;
        const std::chrono::duration<long long, std::nano> render_delta_time_nanoseconds =  render_end_time_nanoseconds - render_start_time_nanoseconds;

        const std::chrono::duration<long double> input_delta_time_seconds = input_delta_time_nanoseconds;
        const std::chrono::duration<long double> physics_delta_time_seconds = physics_delta_time_nanoseconds;
        const std::chrono::duration<long double> gpu_data_update_delta_time_seconds = gpu_data_update_delta_time_nanoseconds;
        const std::chrono::duration<long double> render_delta_time_seconds = render_delta_time_nanoseconds;

        long double everything_else_delta_time_seconds = tick_delta_time_seconds.count() - input_delta_time_seconds.count() - physics_delta_time_seconds.count() - gpu_data_update_delta_time_seconds.count() - render_delta_time_seconds.count();

        input_delta_time_seconds_sum += input_delta_time_seconds.count();
        physics_delta_time_seconds_sum += physics_delta_time_seconds.count();
        gpu_data_update_delta_time_seconds_sum += gpu_data_update_delta_time_seconds.count();
        render_delta_time_seconds_sum += render_delta_time_seconds.count();
        everything_else_delta_time_seconds_sum += everything_else_delta_time_seconds;
        if (time_between_last_FPS_calculation >= time_between_last_FPS_calculation_target) {
            FPS = number_of_frames / time_between_last_FPS_calculation;
            input_delta_time_seconds_average = input_delta_time_seconds_sum / number_of_frames;
            physics_delta_time_seconds_average = physics_delta_time_seconds_sum / number_of_frames;
            gpu_data_update_delta_time_seconds_average = gpu_data_update_delta_time_seconds_sum / number_of_frames;
            render_delta_time_seconds_average = render_delta_time_seconds_sum / number_of_frames;
            everything_else_delta_time_seconds_average = everything_else_delta_time_seconds_sum / number_of_frames;

            number_of_frames = 0;
            time_between_last_FPS_calculation = 0.0;
            input_delta_time_seconds_sum = 0.0;
            physics_delta_time_seconds_sum = 0.0;
            gpu_data_update_delta_time_seconds_sum = 0.0;
            render_delta_time_seconds_sum = 0.0;
            everything_else_delta_time_seconds_sum = 0.0;
        }
        // input
        // -----
        input_start_time_nanoseconds = std::chrono::high_resolution_clock::now();
        processInput(window);
        input_end_time_nanoseconds = std::chrono::high_resolution_clock::now();

        // physics
        // -------
        physics_start_time_nanoseconds = std::chrono::high_resolution_clock::now();
        updatePhysics(physics_timestep);
        updateAllParticlesPositionDataInGPU(*current_particle_model);
        physics_end_time_nanoseconds = std::chrono::high_resolution_clock::now();

        // render
        // ------
        gpu_data_update_start_time_nanoseconds = std::chrono::high_resolution_clock::now();
        updateViewAndProjection();
        gpu_data_update_end_time_nanoseconds = std::chrono::high_resolution_clock::now();

        render_start_time_nanoseconds = std::chrono::high_resolution_clock::now();
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        drawAllParticles(*current_particle_model);
#if DRAW_DEBUG_DATA_IN_WINDOW
        tr.RenderText(std::format("FPS: {}", FPS), 0.1, 0.1, 0.25, glm::vec3{ 1.0f, 1.0f, 1.0f });
        tr.RenderText(std::format("input_delta_time_seconds_average: {}", input_delta_time_seconds_average), 0.1, 25, 0.25, glm::vec3{ 1.0f, 1.0f, 1.0f });
        tr.RenderText(std::format("physics_delta_time_seconds_average: {}", physics_delta_time_seconds_average), 0.1, 50, 0.25, glm::vec3{ 1.0f, 1.0f, 1.0f });
        tr.RenderText(std::format("gpu_data_update_delta_time_seconds_average: {}", gpu_data_update_delta_time_seconds_average), 0.1, 75, 0.25, glm::vec3{ 1.0f, 1.0f, 1.0f });
        tr.RenderText(std::format("render_delta_time_seconds_average: {}", render_delta_time_seconds_average), 0.1, 100, 0.25, glm::vec3{ 1.0f, 1.0f, 1.0f });
        tr.RenderText(std::format("everything_else_delta_time_seconds_average: {}", everything_else_delta_time_seconds_average), 0.1, 125, 0.25, glm::vec3{ 1.0f, 1.0f, 1.0f });
        tr.RenderText(std::format("delta_time: {}", delta_time), 0.1, 150, 0.25, glm::vec3{ 1.0f, 1.0f, 1.0f });
        tr.RenderText(std::format("Middle Particle Collision Count: {}", pm->particles[1].collision_count), 0.1, 175, 0.25, glm::vec3{ 1.0f, 1.0f, 1.0f });
#else
        std::cout << "FPS: " << FPS << '\n';
        std::cout << "input_delta_time_seconds_average: " << input_delta_time_seconds_average << '\n';
        std::cout << "physics_delta_time_seconds_average: " << physics_delta_time_seconds_average << '\n';
        std::cout << "gpu_data_update_delta_time_seconds_average: " << gpu_data_update_delta_time_seconds_average << '\n';
        std::cout << "render_delta_time_seconds_average: " << render_delta_time_seconds_average << '\n';
        std::cout << "everything_else_delta_time_seconds_average: " << everything_else_delta_time_seconds_average << '\n';
        std::cout << "delta_time: " << delta_time << '\n';
        std::cout << "input_delta_time_seconds: " << input_delta_time_seconds.count() << '\n';
        std::cout << "physics_delta_time_seconds: " << physics_delta_time_seconds.count() << '\n';
        std::cout << "gpu_data_update_delta_time_seconds: " << gpu_data_update_delta_time_seconds.count() << '\n';
        std::cout << "render_delta_time_seconds: " << render_delta_time_seconds.count() << '\n';
        std::cout << "everything_else_delta_time_seconds: " << everything_else_delta_time_seconds << '\n';
        std::cout << "Middle Particle Collision Count: " << pm->particles[1].collision_count << '\n';
        printf("\x1b[H");
#endif
        render_end_time_nanoseconds = std::chrono::high_resolution_clock::now();

        // ending stuff
        // ------------
        glfwSwapBuffers(window);
        glfwPollEvents();
        number_of_frames++;
    }
}

void Simulation::updateAllParticlesPositionDataInGPU(Model& particle_model, const glm::vec3 &scale ) const {
    pm->updateDataInGPU(particle_model, scale);
}

void Simulation::drawAllParticles(const Model &particle_model) const {
    particle_model.drawInstanced(*shader);
}

void Simulation::loadDefaultParticleModel() {
    current_particle_model = new Model(1.0, 12);
    pm = new ParticleManager();
}
