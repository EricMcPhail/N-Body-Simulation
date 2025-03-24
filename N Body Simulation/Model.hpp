#pragma once
#include "Mesh.hpp"
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <glm/gtc/matrix_transform.hpp> // for glm::translate


glm::mat4 getTransformation(const glm::vec3& position,
                            const glm::mat4& orientation = glm::mat4(1.0f),
                            const glm::vec3& scale = glm::vec3(1.0f,1.0f,1.0f)) {
    glm::mat4 transformation = orientation;
    transformation = glm::translate(transformation, position);
    transformation = glm::scale(transformation, scale);
    return transformation;
}

glm::mat4 getTransformation(const glm::vec3& position, const glm::vec3& scale) {
    glm::mat4 transformation = glm::mat4(1.0f);
    transformation = glm::translate(transformation, position);
    transformation = glm::scale(transformation, scale);
    return transformation;
}


std::vector<glm::mat4> getTransformations(const std::vector<glm::vec3>& positions,
                                          const std::vector<glm::mat4>& orientations,
                                          const std::vector<glm::vec3>& scales) {
    std::vector<glm::mat4> transformations(positions.size());
    for (size_t i = 0; i < positions.size(); i++) {
        glm::mat4 model = orientations[i];
        model = glm::translate(model, positions[i]);
        model = glm::scale(model, scales[i]);
        transformations.push_back(model);
    }
    return transformations;
}

std::vector<glm::mat4> getTransformations(const std::vector<glm::vec3>& positions,
                                          const std::vector<glm::mat4>& orientations) {
    std::vector<glm::mat4> transformations(positions.size());
    for (size_t i = 0; i < positions.size(); i++) {
        glm::mat4 model = orientations[i];
        model = glm::translate(model, positions[i]);
        transformations.push_back(model);
    }
    return transformations;
}


std::vector<glm::mat4> getTransformations(const std::vector<glm::vec3>& positions,
                                          const std::vector<glm::vec3>& scales) {
    std::vector<glm::mat4> transformations(positions.size());
    for (size_t i = 0; i < positions.size(); i++) {
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, positions[i]);
        model = glm::scale(model, scales[i]);
        transformations.push_back(model);
    }
    return transformations;
}


std::vector<glm::mat4> getTransformations(const std::vector<glm::vec3>& positions,
                                          const glm::mat4& orientation = glm::mat4(1.0f),
                                          const glm::vec3& scale = glm::vec3(1.0f, 1.0f, 1.0f)) {
    std::vector<glm::mat4> transformations(positions.size());
    for (size_t i = 0; i < positions.size(); i++) {
        glm::mat4 model = orientation;
        model = glm::translate(model, positions[i]);
        model = glm::scale(model, scale);
        transformations.push_back(model);
    }
    return transformations;
}


std::vector<glm::mat4> getTransformations(const std::vector<glm::vec3>& positions, const glm::vec3& scale) {
    std::vector<glm::mat4> transformations(positions.size());
    for (size_t i = 0; i < positions.size(); i++) {
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, positions[i]);
        model = glm::scale(model, scale);
        transformations.push_back(model);
    }
    return transformations;
}







enum Shape {
    CIRLCE,
    ELLIPSOID,
    QUAD
};




struct Model {
	std::vector<Mesh> meshes;
    //size_t number_of_instances = 0;



    Model() {}

    // draws the model, and thus all its meshes
    void draw() {
        for (unsigned int i = 0; i < meshes.size(); i++)
            meshes[i].draw();
    }

    void draw(const glm::mat4& transformation) {
        for (unsigned int i = 0; i < meshes.size(); i++)
            meshes[i].updateAndDraw(transformation);
    }

    void drawInstanced() {
        for (unsigned int i = 0; i < meshes.size(); i++)
            meshes[i].drawInstanced();
    }

    void drawInstanced(const std::vector<glm::mat4>& transformations) {
        for (unsigned int i = 0; i < meshes.size(); i++)
            meshes[i].updateAndDrawInstanced(transformations);
    }

    Model(std::string const& path) {
        // Todo

    }

    Model(Shape shape) {
        Mesh mesh;
        switch (shape) {
        case Shape::CIRLCE:
            mesh.buildCircle();
            meshes.push_back(mesh);
            break;
        case Shape::ELLIPSOID:
            mesh.buildEllipsoid();
            meshes.push_back(mesh);
            break;
        case Shape::QUAD:
            break;
        default:
            throw - 1;
        }

    }




};




