#pragma once
#include "MathFunctions.hpp"
#include <glm/glm.hpp>

template<size_t number_of_dimensions, typename FloatingPointType> struct Box {
	glm::vec<number_of_dimensions, FloatingPointType> min;
	glm::vec<number_of_dimensions, FloatingPointType> max;


};


template<size_t N, typename T> struct AABB {
	Box A<N, T>;
	Box B<N, T>;
	Box<N, T> minkowskiDifference() {
		return Box<N, T>{A.min - B.max, A.max - B.min};
	}
	double timeTillCollision() {
		Box<N, T> minkowski_difference = minkowskiDifference();
		bool result = true;
		for (size_t i = 0; i < N; i++) {
			result = result && minkowski_difference.min[i] <= 0;
			result = result && minkowski_difference.max[i] >= 0;
		}
		if (result) {
			// normal discrete collision detection / separation code
			// already colliding?
			// check if alreadt colliding
			return 0.0;
		}

		// calculate the relative motion between the two boxes
		var relativeMotion : Vector = (boxA.velocity - boxB.velocity) * dt;

		// ray-cast the relativeMotion vector against the Minkowski AABB
		var h : Float = md.getRayIntersectionFraction(Vector.zero, relativeMotion);

		// check to see if a collision will happen this frame
		// getRayIntersectionFraction returns Math.POSITIVE_INFINITY if there is no intersection
		if (h < Math.POSITIVE_INFINITY)
		{
			// yup, there WILL be a collision this frame
			// move the boxes appropriately
			boxA.center += boxA.velocity * dt * h;
			boxB.center += boxB.velocity * dt * h;

			// zero the normal component of the velocity
			// (project the velocity onto the tangent of the relative velocities
			//  and only keep the projected component, tossing the normal component)
			var tangent : Vector = relativeMotion.normalized.tangent;
			boxA.velocity = Vector.dotProduct(boxA.velocity, tangent) * tangent;
			boxB.velocity = Vector.dotProduct(boxB.velocity, tangent) * tangent;
		}
		else
		{
			// no intersection, move it along
			boxA.center += boxA.velocity * dt;
			boxB.center += boxB.velocity * dt;
		}


	}

};


#include <limits>

// taken from https://github.com/pgkelley4/line-segments-intersect/blob/master/js/line-segments-intersect.js
// which was adapted from http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
// returns the point where they intersect (if they intersect)
// returns Math.POSITIVE_INFINITY if they don't intersect
double getRayIntersectionFractionOfFirstRay(glm::vec3  originA, glm::vec3  endA , glm::vec3  originB , glm::vec3  endB )
{
	glm::vec3 r  = endA - originA;
	glm::vec3 s  = endB - originB;

	double numerator = glm::dot((originB - originA), r);
	double denominator = glm::dot(r ,s );

	if (numerator == 0 && denominator == 0)
	{
		// the lines are co-linear
		// check if they overlap
		/*return	((originB.x - originA.x < 0) != (originB.x - endA.x < 0) != (endB.x - originA.x < 0) != (endB.x - endA.x < 0)) ||
				((originB.y - originA.y < 0) != (originB.y - endA.y < 0) != (endB.y - originA.y < 0) != (endB.y - endA.y < 0));*/
		return std::numeric_limits<double>::infinity();
	}
	if (denominator == 0)
	{
		// lines are parallel
		return std::numeric_limits<double>::infinity();
	}

	double u = numerator / denominator;
	double t  = glm::dot((originB - originA), s) / denominator;
	if ((t >= 0) && (t <= 1) && (u >= 0) && (u <= 1))
	{
		//return originA + (r * t);
		return t;
	}
	return std::numeric_limits<double>::infinity();
}

double getRayIntersectionFraction(glm::vec3 origin, glm::vec3 direction, glm::vec3 min, glm::vec3 max) {
	glm::vec3 end = origin + direction;

	double  minT = getRayIntersectionFractionOfFirstRay(origin, end, glm::vec3(min.x, min.y), glm::vec3(min.x, max.y));
	double x;
	x = getRayIntersectionFractionOfFirstRay(origin, end, glm::vec3(min.x, max.y), glm::vec3(max.x, max.y));
	if (x < minT) minT = x;
	x = getRayIntersectionFractionOfFirstRay(origin, end, glm::vec3(max.x, max.y), glm::vec3(max.x, min.y));
	if (x < minT) minT = x;
	x = getRayIntersectionFractionOfFirstRay(origin, end, glm::vec3(max.x, min.y), glm::vec3(min.x, min.y));
	if (x < minT) minT = x;

	// ok, now we should have found the fractional component along the ray where we collided
	return minT;
}


template<size_t N, typename T> double getRayIntersectionFraction(glm::vec3 origin, glm::vec3 direction, glm::vec3 min, glm::vec3 max) {
	glm::vec3 end = origin + direction;
	size_t num_possibilities = power_of_two(N);
	glm::vec<N, T> param1;
	for (size_t i = 0; i < N; i++) param1[i] = min[i];
	glm::vec<N, T> param2;
	for (size_t i = 0; i < (N - 1); i++) param2[i] = min[i];
	param2[N - 1] = max[N - 1];


	double  minT = getRayIntersectionFractionOfFirstRay(origin, end, param1, param2);
	double x;
	for (size_t i = 0; i < num_possibilities; i++) {
		
		x = getRayIntersectionFractionOfFirstRay(origin, end, glm::vec<N,T>(min.x, max.y), glm::vec<N, T>(max.x, max.y));
		if (x < minT) minT = x;
	}


	
	
	x = getRayIntersectionFractionOfFirstRay(origin, end, glm::vec3(max.x, max.y), glm::vec3(max.x, min.y));
	if (x < minT) minT = x;
	x = getRayIntersectionFractionOfFirstRay(origin, end, glm::vec3(max.x, min.y), glm::vec3(min.x, min.y));
	if (x < minT) minT = x;

	// ok, now we should have found the fractional component along the ray where we collided
	return minT;
}

