#pragma once
#include <vector>
#include <glm/glm.hpp>

class Shader;
struct Vertex {
    glm::vec3 position;
    glm::vec3 colour;
};

class Mesh {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    Shader* shader = nullptr;
    size_t number_of_instances = 0;

    bool is_loaded_in_gpu = false;
    bool has_element_buffer = false;
    bool use_wireframe = false;
    unsigned int drawing_mode;

    unsigned int vertex_array_object_ID; // VAO
    unsigned int vertex_buffer_object_ID; // VBO
    unsigned int element_buffer_object_ID; // EBO
    unsigned int instance_vertex_buffer_object_ID; // IVBO
public:
    Mesh();
    Mesh(std::vector<Vertex> vertices);
    Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices);
    ~Mesh();

    void createBuffer();

    // UGH WTF IS THIS, WHUY DO I HAVE SO MANY DRAW OPTIONS, DUMB REALLY DUMB
    void updateData(const glm::mat4& transformation);
    void updateData(const glm::vec3& position, const glm::vec3& scale, const glm::mat4& orientation);
    void updateData(const glm::vec3& position, const glm::vec3& scale, const glm::vec3& angles);
    void updateData(const glm::vec3& position);
    void updateInstancedData(const std::vector<glm::mat4>& transformations);
    void updateInstancedData(const std::vector<glm::vec3>& positions, const std::vector<glm::vec3>& scales, const std::vector<glm::mat4>& orientations);
    void updateInstancedData(const std::vector<glm::vec3>& positions, const std::vector<glm::vec3>& scales, const std::vector<glm::vec3>& angles);
    void updateInstancedData(const std::vector<glm::vec3>& positions);

    void draw() const;
    void drawInstanced() const;
    // when the fuck do i use these, why do i have these lmfao kill me
    void updateAndDraw(const glm::mat4& transformation);
    void updateAndDrawInstanced(const std::vector<glm::mat4>& transformations);

    // Construct vertices for basic shapes
    void buildEllipsoid(float a = 1.0f, float b = 2.0f, float c = 3.0f, size_t stacks = 15, size_t slices = 15, const glm::vec3& colour = glm::vec3{ 1.0f, 1.0f, 1.0f });
    void buildCircle(float radius = 1.0f, size_t num_triangles = 30, const glm::vec3& colour = glm::vec3{ 1.0f, 1.0f, 1.0f });
    void buildAABB(glm::vec3& min, glm::vec3& max, const glm::vec3& colour = glm::vec3{ 1.0f, 1.0f, 1.0f });
};
