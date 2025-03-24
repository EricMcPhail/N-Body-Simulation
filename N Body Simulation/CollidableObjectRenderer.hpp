#pragma once
#include "CollidableObject.hpp"
#include "Model.hpp"

inline glm::vec3 translateToScreenSpace(const VectorND& v) {
    switch (VECTOR_SPACE_NUMBER_OF_DIMENSIONS) {
    case 0:
        return glm::vec3{ 0.0f, 0.0f, 0.0f };
    case 1:
        return glm::vec3{
            static_cast<float>(v[0]),
            0.0f,
            0.0f };
    case 2:
        return glm::vec3{
            static_cast<float>(v[0]),
            static_cast<float>(v[1]),
            0.0f };
    case 3:
        return glm::vec3{
            static_cast<float>(v[0]),
            static_cast<float>(v[1]),
            static_cast<float>(v[2]) };
    default:
        // TODO: FIGURE OUT WHAT TO DO IN THE N > 3 CASE
        return glm::vec3{
            static_cast<float>(v[0]),
            static_cast<float>(v[1]),
            static_cast<float>(v[2]) };
    }

}



struct CollidableObjectModelPair {
	Model* model;
	std::vector<CollidableObject*> collidable_objects;
	std::vector<glm::vec3> getPositions() {
		std::vector<glm::vec3> ret_val(collidable_objects.size());
		for (const auto& x : collidable_objects) {
			ret_val.push_back(translateToScreenSpace(x->position));
		}

		return ret_val;
	}
};

struct CollidableObjectRenderer {
	std::vector<CollidableObjectModelPair> collidable_objects_model_pair;
	// we have all the positions of the objects
	// we have all the models? they just need to be orginized for one draw call


	void draw() {
		for (const auto& render_data : collidable_objects_model_pair) {

			const std::vector<glm::mat4> transformations = getTransformations();
			render_data.model->drawInstanced(transformations);
		}
	}
};