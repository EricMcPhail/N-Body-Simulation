#pragma once
#include <vector>
#include <glm/glm.hpp>

class Shader;
struct Vertex;


// TODO: make this a singleton class, I fucing am so lazy
class Model {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    size_t number_of_instances = 0;
    bool is_loaded_in_gpu = false;
    bool has_element_buffer = false;
    unsigned int drawingMode;

    unsigned int vertex_array_object_ID; // VAO
    unsigned int vertex_buffer_object_ID; // VBO
    unsigned int element_buffer_object_ID; // EBO
    unsigned int instance_vertex_buffer_object_ID; // IVBO
public:
    Model(float raduis = 1.0, size_t number_of_triangles = 10, const glm::vec3& colour = glm::vec3{ 0.5, 0.7, 0.2 });
    Model(float a, float b, float c, size_t stacks, size_t slices, const glm::vec3& colour);
    ~Model();

    void createBuffer(bool add_element_buffer);
    //tt

    void updateData(const glm::mat4& transformations);
    void updateInstancedData(const std::vector<glm::mat4>& transformations);
    void draw(const Shader& shader, bool wire_frame = true) const;
    void drawInstanced(const Shader& shader, bool wire_frame = true) const;
    void updateAndDraw(const glm::mat4& transformation, Shader& shader);
    void updateAndDrawInstanced(const std::vector<glm::mat4>& transformations, Shader& shader);

    void buildEllipsoid(float a, float b, float c, size_t stacks, size_t slices, const glm::vec3& colour);
    void buildCircle(float radius, size_t num_triangles, const glm::vec3& colour = glm::vec3{ 1.0f, 1.0f, 1.0f });
};
