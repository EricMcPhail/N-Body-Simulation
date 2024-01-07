#pragma once
#include <vector>

#include <glm/glm.hpp>

extern class Shader;
extern struct Vertex;


// TODO: make this a singleton class, I fucing am so lazy
class Model {
    std::vector<Vertex> vertices;
    size_t number_of_instances = 0;
    bool is_loaded_in_gpu = false;
    unsigned int vertex_array_object_ID;
    unsigned int vertex_buffer_object_ID;
    unsigned int instance_vertex_buffer_object_ID;
public:
    Model(float raduis = 1.0, size_t number_of_triangles = 10, const glm::vec3& colour = glm::vec3{ 0.5, 0.7, 0.2 });

    ~Model();

    void buildCircle(float radius, size_t num_triangles, const glm::vec3 & colour = glm::vec3{ 1.0f, 1.0f, 1.0f });

    void createBuffer();
    //tt

    void updateData(const glm::mat4& transformations);
    void updateInstancedData(const std::vector<glm::mat4>& transformations);
    void draw(const Shader& shader, bool wire_frame = false) const;
    void drawInstanced(const Shader& shader, bool wire_frame = false) const;
    void updateAndDraw(const glm::mat4& transformation, Shader& shader);
    void updateAndDrawInstanced(const std::vector<glm::mat4>& transformations, Shader& shader);
};
