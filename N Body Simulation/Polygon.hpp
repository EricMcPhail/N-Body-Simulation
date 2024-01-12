#pragma once
#include "MathFunctions.hpp"


#include <glm/glm.hpp>
#include <vector>
#include <type_traits>



template<typename... Ts> using AllDoubles = typename std::conjunction<std::is_same<Ts, double>...>::type;

#define DEFAULT_DIMENSION_LENGTH 1.0

#define DEFAULT_SIMPLEX_EDGE_LENGTH 1.0

const double ONE_OVER_ROOT_TWO = 1.0 / sqrt(2.0);
const vec3 vector_of_ones(1.0, 1.0, 1.0);

using vec3 = glm::highp_dvec3;





class Geometry {
public:
	vec3 position;
	static std::vector<vec3> basis_vectors;

	Geometry() {
		basis_vectors.push_back(vec3(1.0, 0.0, 0.0));
		basis_vectors.push_back(vec3(0.0, 1.0, 0.0));
		basis_vectors.push_back(vec3(0.0, 0.0, 1.0));
	}

};

class N_Sphere : public Geometry {
public:
	double radius;
};

class N_ConvexPolygon : public Geometry {
public:
	
	std::vector<vec3> vertices;
	std::vector<std::pair<size_t, size_t>> edges;
	vec3 orientation;
	size_t number_of_dimensions;



	N_ConvexPolygon() {}
};

class N_Simplex : public N_ConvexPolygon {
public:
	N_Simplex(const std::vector<vec3>& points) {
		number_of_dimensions = points.size() - 1;
		assert(!points.empty());
		vertices.reserve(number_of_dimensions + 1);
		size_t index = 0;
		for (size_t i = 0; i <= number_of_dimensions; i++) {
			vertices[i] = points[i];
			for (size_t j = i + 1; j <= number_of_dimensions; j++) {
				edges.push_back(std::pair<size_t, size_t>(i, j));
			}
		}
	}
	
	N_Simplex(size_t N) {
		number_of_dimensions = N;
		assert(number_of_dimensions); // Must be greater or equal to 1
		vertices.reserve(number_of_dimensions + 1);
		for (size_t i = 0; i < number_of_dimensions; i++) {
			vertices[i] = ONE_OVER_ROOT_TWO * basis_vectors[i] - (1.0 / number_of_dimensions) * ONE_OVER_ROOT_TWO * (1.0 + 1.0 / sqrt(number_of_dimensions + 1)) * vector_of_ones;
			for (size_t j = i + 1; j <= number_of_dimensions; j++) {
				edges.push_back(std::pair<size_t, size_t>(i, j));
			}
		}
		vertices[number_of_dimensions] = 1.0 / sqrt(2.0 * (number_of_dimensions + 1)) * vector_of_ones;
	}

};

class N_Quad : public N_ConvexPolygon {
	void createVerticesAndEdges(size_t current_dimension = 1) {
		assert(number_of_dimensions); // Must be greater or equal to 1
		if (current_dimension == 1) {
			vertices[0][0] -= dimension_lengths[0] / 2.0;
			vertices[1][0] += dimension_lengths[0] / 2.0;
			edges.push_back(std::pair<size_t, size_t>(0,1));
		}
		if (current_dimension == number_of_dimensions) return;

		size_t current_number_of_vertices = power_of_two(current_dimension);
		size_t current_number_of_dimensions = current_dimension;

		for (size_t i = 0; i < current_number_of_vertices; i++) {
			int new_vertex_index = current_number_of_vertices + i;
			for (size_t j = 0; j < current_number_of_dimensions; j++) {
				vertices[new_vertex_index][j] = vertices[i][j];
			}
			vertices[i][current_number_of_dimensions] -= dimension_lengths[current_dimension] / 2.0;
			vertices[new_vertex_index][current_number_of_dimensions] += dimension_lengths[current_dimension] / 2.0;
			edges.push_back(std::pair<size_t, size_t>(i, new_vertex_index));
		}

		createVerticesAndEdges(current_dimension + 1);
	}

public:
	std::vector<double> dimension_lengths; // length (x), width (y), height (z), etc.

	template<typename... T> N_Quad(T... args) :
		number_of_dimensions{ sizeof...(T) } {
		static_assert(AllDoubles<T...>::value, "All parameters must be doubles!!!\n");
		dimension_lengths.reserve(number_of_dimensions);
		vertices.reserve(power_of_two(number_of_dimensions));
		size_t index = 0;
		for (const auto dimension_length : { args... }) {
			dimension_lengths[index] = dimension_length;
			index++;
		}
		for (size_t i = index; i < number_of_dimensions; i++) {
			dimension_lengths[i] = DEFAULT_DIMENSION_LENGTH;
		}
		createVerticesAndEdges();
	}

};

