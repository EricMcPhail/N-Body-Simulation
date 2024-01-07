#include "Model.hpp"
#include "Shader.hpp"
#include "Constants.hpp" // needed for pi
#include <cassert>

struct Vertex {
    glm::vec3 position;
    glm::vec3 colour;
};


Model::Model(float raduis , size_t number_of_triangles , const glm::vec3&  colour) {
    buildCircle(raduis, number_of_triangles, colour);
    createBuffer();
}

Model::~Model() {
    glDeleteVertexArrays(1, &vertex_array_object_ID);
    glDeleteBuffers(1, &vertex_buffer_object_ID);
    glDeleteBuffers(1, &instance_vertex_buffer_object_ID);
}

void Model::buildCircle(float radius, size_t num_triangles, const glm::vec3& colour) {
    assert(num_triangles >= 3);
    const glm::vec3 center = glm::vec3{ 0.0, 0.0, 0.0 };

    for (size_t i = 0; i < num_triangles; i++) {

        vertices.push_back(Vertex{ center, colour });


        float angle = ((float)i / (float)num_triangles) * (2.0 * PI);
        float x = radius * cos(angle);
        float y = radius * sin(angle);
        float z = 0.0f;
        vertices.push_back(Vertex{ glm::vec3{x, y, z}, colour });


        angle = ((float)(i + 1) / (float)num_triangles) * (2.0 * PI);
        x = radius * cos(angle);
        y = radius * sin(angle);
        z = 0.0f;

        vertices.push_back(Vertex{ glm::vec3{x, y, z}, colour });
    }
}

void Model::createBuffer() {
    glGenVertexArrays(1, &vertex_array_object_ID);
    glGenBuffers(1, &vertex_buffer_object_ID);
    glGenBuffers(1, &instance_vertex_buffer_object_ID);

    // Send the Vertex data to the GPU
    glBindVertexArray(vertex_array_object_ID);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_object_ID);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW); // Using Static_Draw because we dont care to modify the vertices

    // Position attribute of the Vertex
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);

    // Colour attribute of the Vertex
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, colour));

    // Linear transformation attribute
    glEnableVertexAttribArray(2);
    glBindBuffer(GL_ARRAY_BUFFER, instance_vertex_buffer_object_ID); // this attribute comes from a different vertex buffer

    // set attribute pointers for matrix (4 times vec4)
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (void*)0);
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (void*)(sizeof(glm::vec4)));
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (void*)(2 * sizeof(glm::vec4)));
    glEnableVertexAttribArray(5);
    glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (void*)(3 * sizeof(glm::vec4)));

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    // tell OpenGL these are instanced vertex attributes.
    glVertexAttribDivisor(2, 1);
    glVertexAttribDivisor(3, 1);
    glVertexAttribDivisor(4, 1);
    glVertexAttribDivisor(5, 1);
    glBindVertexArray(0);
}

void Model::updateData(const glm::mat4& transformation) {
    glBindBuffer(GL_ARRAY_BUFFER, instance_vertex_buffer_object_ID);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::mat4), &transformation, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    number_of_instances = 1;
}

void Model::updateInstancedData(const std::vector<glm::mat4>& transformations) {
    glBindBuffer(GL_ARRAY_BUFFER, instance_vertex_buffer_object_ID);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::mat4) * transformations.size(), &transformations[0], GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    number_of_instances = transformations.size();
}

void Model::draw(const Shader& shader) const {
    shader.use();
    glBindVertexArray(vertex_array_object_ID);
    glDrawArrays(GL_TRIANGLE_FAN, 0, vertices.size());
    glBindVertexArray(0);
}

void Model::drawInstanced(const Shader& shader) const {
    shader.use();
    glBindVertexArray(vertex_array_object_ID);
    glDrawArraysInstanced(GL_TRIANGLE_FAN, 0, vertices.size(), number_of_instances);
    glBindVertexArray(0);
}

void Model::updateAndDraw(const glm::mat4& transformation, Shader& shader) {
    updateData(transformation);
    draw(shader);
}

void Model::updateAndDrawInstanced(const std::vector<glm::mat4>& transformations, Shader& shader) {
    updateInstancedData(transformations);
    drawInstanced(shader);
}