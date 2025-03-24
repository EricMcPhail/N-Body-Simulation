#include "Mesh.hpp"
#include "Shader.hpp"
#include "Constants.hpp" // needed for pi
#include <cassert>
#include <glm/gtc/matrix_transform.hpp> // for glm::translate


Mesh::Mesh() {}

Mesh::Mesh(std::vector<Vertex> vertices) {
    this->vertices = vertices;
    createBuffer();
}

Mesh::Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices) {
    this->vertices = vertices;
    this->indices = indices;
    createBuffer();
}

Mesh::~Mesh() {
    if (is_loaded_in_gpu) {
        glDeleteVertexArrays(1, &vertex_array_object_ID);
        glDeleteBuffers(1, &vertex_buffer_object_ID);
        if (has_element_buffer) glDeleteBuffers(1, &element_buffer_object_ID);
        glDeleteBuffers(1, &instance_vertex_buffer_object_ID);
    }
}

void Mesh::createBuffer() {
    assert(!vertices.empty());
    assert(is_loaded_in_gpu == false);


    has_element_buffer = !indices.empty();
    glGenVertexArrays(1, &vertex_array_object_ID);
    glGenBuffers(1, &vertex_buffer_object_ID);
    if (has_element_buffer) glGenBuffers(1, &element_buffer_object_ID);
    glGenBuffers(1, &instance_vertex_buffer_object_ID);

    glBindVertexArray(vertex_array_object_ID);

    // Send the Vertex data to the GPU
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_object_ID);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW); // Using Static_Draw because we dont care to modify the vertices

    // Send the Index data to the GPU
    if (has_element_buffer) {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, element_buffer_object_ID);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);
    }

    // Set the vertex attribute pointers
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
    is_loaded_in_gpu = true;
}

void Mesh::updateData(const glm::mat4& transformation) {
    glBindBuffer(GL_ARRAY_BUFFER, instance_vertex_buffer_object_ID);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::mat4), &transformation, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    number_of_instances = 1;
}

void Mesh::updateData(const glm::vec3& position, const glm::vec3& scale, const glm::mat4& orientation) {
    glm::mat4 transformation = orientation;
    transformation = glm::translate(transformation, position);
    transformation = glm::scale(transformation, scale);
    updateData(transformation);
}

void Mesh::updateData(const glm::vec3& position) {
    glm::mat4 transformation = glm::mat4(1.0f);
    transformation = glm::translate(transformation, position);
    updateData(transformation);
}

void Mesh::updateInstancedData(const std::vector<glm::vec3>& positions, const std::vector<glm::vec3>& scales, const std::vector<glm::mat4>& orientations) {
    std::vector<glm::mat4> transformations(positions.size());
    for (size_t i = 0; i < positions.size(); i++) {
        glm::mat4 model = orientations[i];
        model = glm::translate(model, positions[i]);
        model = glm::scale(model, scales[i]);
        transformations.push_back(model);
    }
    updateInstancedData(transformations);
}

void Mesh::updateInstancedData(const std::vector<glm::vec3>& positions) {
    std::vector<glm::mat4> transformations(positions.size());
    for (size_t i = 0; i < positions.size(); i++) {
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, positions[i]);
        transformations.push_back(model);
    }
    updateInstancedData(transformations);
}

void Mesh::updateInstancedData(const std::vector<glm::mat4>& transformations) {
    glBindBuffer(GL_ARRAY_BUFFER, instance_vertex_buffer_object_ID);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::mat4) * transformations.size(), &transformations[0], GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    number_of_instances = transformations.size();
}

void Mesh::draw() const {
    const bool keep_GL_DEPTH_TEST_enabled = glIsEnabled(GL_DEPTH_TEST);
    glEnable(GL_DEPTH_TEST);
    if (use_wireframe) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    shader->use();
    glBindVertexArray(vertex_array_object_ID);

    if (has_element_buffer) glDrawElements(drawing_mode, indices.size(), GL_UNSIGNED_INT, 0);
    else glDrawArrays(drawing_mode, 0, vertices.size());
    glBindVertexArray(0);

    if (!keep_GL_DEPTH_TEST_enabled) glDisable(GL_DEPTH_TEST);
    if (use_wireframe) glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void Mesh::drawInstanced() const {
    const bool keep_GL_DEPTH_TEST_enabled = glIsEnabled(GL_DEPTH_TEST);
    glEnable(GL_DEPTH_TEST);
    if (use_wireframe) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    shader->use();
    glBindVertexArray(vertex_array_object_ID);
    if (has_element_buffer) glDrawElementsInstanced(drawing_mode, indices.size(), GL_UNSIGNED_INT, 0, number_of_instances);
    else glDrawArraysInstanced(drawing_mode, 0, vertices.size(), number_of_instances);
    glBindVertexArray(0);

    if (!keep_GL_DEPTH_TEST_enabled) glDisable(GL_DEPTH_TEST);
    if (use_wireframe) glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void Mesh::updateAndDraw(const glm::mat4& transformation) {
    updateData(transformation);
    draw();
}

void Mesh::updateAndDrawInstanced(const std::vector<glm::mat4>& transformations) {
    updateInstancedData(transformations);
    drawInstanced();
}

void Mesh::buildCircle(float radius, size_t num_triangles, const glm::vec3& colour) {
    drawing_mode = GL_TRIANGLE_FAN;
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

void Mesh::buildEllipsoid(float a, float b, float c, size_t stacks, size_t slices, const glm::vec3& colour) {
    drawing_mode = GL_TRIANGLES;
    has_element_buffer = true;
    for (int i = 0; i <= stacks; ++i) {
        // V texture coordinate
        float V = i / (float)stacks;
        float phi = V * PI - PI / 2.0;
        for (int j = 0; j <= slices; ++j) {

            // U texture coordinate
            float U = j / (float)slices;
            float theta = U * 2.0 * PI;

            float X = a * cos(phi) * cos(theta);
            float Y = b * cos(phi) * sin(theta);
            float Z = c * sin(phi);

            vertices.push_back(Vertex{ glm::vec3(X, Y, Z), colour });

        }
    }


    // creating index buffer
    int noPerSlice = slices + 1;
    for (int i = 0; i < stacks; ++i) {
        for (int j = 0; j < slices; ++j) {

            int start_i = (i * noPerSlice) + j;

            indices.push_back(start_i);
            indices.push_back(start_i + noPerSlice + 1);
            indices.push_back(start_i + noPerSlice);

            indices.push_back(start_i + noPerSlice + 1);
            indices.push_back(start_i);
            indices.push_back(start_i + 1);
        }
    }
}

void buildAABB(glm::vec3& min, glm::vec3& max, const glm::vec3& colour) {

}