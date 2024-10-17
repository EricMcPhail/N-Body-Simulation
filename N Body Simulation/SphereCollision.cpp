#include "SphereCollision.hpp"
#include "VectorSpace.hpp"

bool NSphereCollisionCheckStatic(const VectorND& A_position,
    const double A_radius,
    const VectorND& B_position,
    const double B_radius) {
    const VectorND relative_position_AB = A_position - B_position;
    const double radii_sum = A_radius + B_radius;
    const double radii_sum_sqrd = radii_sum * radii_sum;
    const double distance_between_centers_sqrd = relative_position_AB.dot(relative_position_AB);
    return (distance_between_centers_sqrd <= radii_sum_sqrd);
}

bool NSphereCollisionCheckDynamic(const VectorND& A_position,
    const VectorND& A_velocity,
    const double A_radius,
    const VectorND& B_position,
    const VectorND& B_velocity,
    const double B_radius) {
    const VectorND relative_position_AB = A_position - B_position;
    const double radii_sum = A_radius + B_radius;
    const double radii_sum_sqrd = radii_sum * radii_sum;
    const double distance_between_centers_sqrd = relative_position_AB.dot(relative_position_AB);
    if (distance_between_centers_sqrd <= radii_sum_sqrd) {
        return true; // already touching or intercecting
    }

    const VectorND relative_velocity_AB = A_velocity - B_velocity;
    const double a = relative_velocity_AB.dot(relative_velocity_AB);
    if (a == 0.0) {
        // two spheres are moving exactly same dir and speed, not touching or intersecting
        return false;
    }
    const double b = 2.0 * relative_position_AB.dot(relative_velocity_AB);
    const double c = distance_between_centers_sqrd - radii_sum_sqrd;
    const double b_sqrd = b * b;
    const double temp_coeff_1 = b_sqrd - 4.0 * a * c;
    if (temp_coeff_1 < 0.0) {
        // two spheres are not on a colliding path???
        return false;
    }
    double temp_coeff_2 = sqrt(temp_coeff_1);
    double fraction = 1.0 / (2.0 * a); // Since a is strictly positive, fraction is also strictly positve
    double larger_value = (-b + temp_coeff_2) * fraction;
    double smaller_value = (-b - temp_coeff_2) * fraction;

    assert(!(smaller_value <= 0.0 && larger_value >= 0.0)); // we already would of been colliding to start with
    assert(larger_value >= smaller_value);

    if (smaller_value > 0.0) return true;
    else return false;
}

bool timeUntillNSphereCollision(const VectorND& A_position,
    const VectorND& A_velocity,
    const double A_radius,
    const VectorND& B_position,
    const VectorND& B_velocity,
    const double B_radius,
    double& intersection_time_start) {
    VectorND relative_position_AB = A_position - B_position;
    double radii_sum = A_radius + B_radius;
    double radii_sum_sqrd = radii_sum * radii_sum;
    double distance_between_centers_sqrd = relative_position_AB.dot(relative_position_AB);

    if (distance_between_centers_sqrd <= radii_sum_sqrd) {
        intersection_time_start = 0.0;
        return true;
    }

    VectorND relative_velocity_AB = A_velocity - B_velocity;
    const double a = relative_velocity_AB.dot(relative_velocity_AB);
    if (a == 0.0) {
        // two spheres are moving exactly same dir and speed, not touching or intersecting
        intersection_time_start = -std::numeric_limits<double>::infinity();
        return false;
    }
    [[assume(a > 0.0)]];
    double b = 2.0 * relative_position_AB.dot(relative_velocity_AB);
    double c = distance_between_centers_sqrd - radii_sum_sqrd;
    double b_sqrd = b * b;
    double temp_coeff_1 = b_sqrd - 4.0 * a * c;
    if (temp_coeff_1 < 0.0) {
        intersection_time_start = std::numeric_limits<double>::quiet_NaN();
        return false;
    }
    double temp_coeff_2 = sqrt(temp_coeff_1);
    double fraction = 1.0 / (2.0 * a); // Since a is strictly positive, fraction is also strictly positve

    double smaller_value = (-b - temp_coeff_2) * fraction;

    intersection_time_start = smaller_value;

    if (smaller_value > 0.0) return true;
    else return false;
}

bool timeUntillNSphereCollision(const VectorND& A_position,
    const VectorND& A_velocity,
    const double A_radius,
    const VectorND& B_position,
    const VectorND& B_velocity,
    const double B_radius,
    double& intersection_time_start) {
    VectorND relative_position_AB = A_position - B_position;
    double radii_sum = A_radius + B_radius;
    double radii_sum_sqrd = radii_sum * radii_sum;
    double distance_between_centers_sqrd = relative_position_AB.dot(relative_position_AB);

    if (distance_between_centers_sqrd <= radii_sum_sqrd) {
        intersection_time_start = 0.0;
        return true;
    }

    VectorND relative_velocity_AB = A_velocity - B_velocity;
    double a = relative_velocity_AB.dot(relative_velocity_AB);
    if (a == 0.0) {
        // two spheres are moving exactly same dir and speed, not touching or intersecting
        intersection_time_start = -std::numeric_limits<double>::infinity();
        return false;
    }
    double b = 2.0 * relative_position_AB.dot(relative_velocity_AB);
    double c = distance_between_centers_sqrd - radii_sum_sqrd;
    double b_sqrd = b * b;
    double temp_coeff_1 = b_sqrd - 4.0 * a * c;
    if (temp_coeff_1 < 0.0) {
        intersection_time_start = std::numeric_limits<double>::quiet_NaN();
        return false;
    }
    double temp_coeff_2 = sqrt(temp_coeff_1);
    double fraction = 1.0 / (2.0 * a); // Since a is strictly positive, fraction is also strictly positve

    double smaller_value = (-b - temp_coeff_2) * fraction;

    intersection_time_start = smaller_value;
    return (smaller_value > 0.0);
}

bool timeUntillNSphereCollision(const VectorND& A_position,
    const VectorND& A_velocity,
    const double A_radius,
    const VectorND& B_position,
    const VectorND& B_velocity,
    const double B_radius,
    double& intersection_time_start,
    double& intersection_time_end) {
    VectorND relative_position_AB = A_position - B_position;
    double radii_sum = A_radius + B_radius;
    double radii_sum_sqrd = radii_sum * radii_sum;
    double distance_between_centers_sqrd = relative_position_AB.dot(relative_position_AB);

    if (distance_between_centers_sqrd <= radii_sum_sqrd) {
        intersection_time_start = 0.0;
        VectorND relative_velocity_AB = A_velocity - B_velocity;
        double a = relative_velocity_AB.dot(relative_velocity_AB);
        if (a == 0.0) {
            intersection_time_end = std::numeric_limits<double>::infinity();
        }
        else {
            double b = 2.0 * relative_position_AB.dot(relative_velocity_AB);
            double c = distance_between_centers_sqrd - radii_sum_sqrd;
            double b_sqrd = b * b;
            double temp_coeff_1 = b_sqrd - 4.0 * a * c;
            double temp_coeff_2 = sqrt(temp_coeff_1);
            double fraction = 1.0 / (2.0 * a);
            double larger_value = (-b + temp_coeff_2) * fraction;
            intersection_time_end = larger_value;
        }
        return true; // already touching or intercecting
    }

    VectorND relative_velocity_AB = A_velocity - B_velocity;
    double a = relative_velocity_AB.dot(relative_velocity_AB);
    if (a == 0.0) {
        // two spheres are moving exactly same dir and speed, not touching or intersecting
        intersection_time_start = -std::numeric_limits<double>::infinity();
        intersection_time_end = std::numeric_limits<double>::infinity();
        return false;
    }
    double b = 2.0 * relative_position_AB.dot(relative_velocity_AB);
    double c = distance_between_centers_sqrd - radii_sum_sqrd;
    double b_sqrd = b * b;
    double temp_coeff_1 = b_sqrd - 4.0 * a * c;
    if (temp_coeff_1 < 0.0) {
        intersection_time_start = std::numeric_limits<double>::quiet_NaN();
        intersection_time_end = std::numeric_limits<double>::quiet_NaN();
        return false;
    }
    double temp_coeff_2 = sqrt(temp_coeff_1);
    double fraction = 1.0 / (2.0 * a); // Since a is strictly positive, fraction is also strictly positve

    double larger_value = (-b + temp_coeff_2) * fraction;
    double smaller_value = (-b - temp_coeff_2) * fraction;

    intersection_time_start = smaller_value;
    intersection_time_end = larger_value;

    assert(!(smaller_value <= 0.0 && larger_value >= 0.0)); // we already would of been colliding to start with
    assert(larger_value >= smaller_value);

    if (smaller_value > 0.0) return true;
    else return false;
}

bool timeUntillNSphereCollisionAccurate(const VectorND& A_position,
    const VectorND& A_velocity,
    const VectorND& A_acceleration,
    const double A_radius,
    const VectorND& B_position,
    const VectorND& B_velocity,
    const VectorND& B_acceleration,
    const double B_radius,
    double& intersection_time_start) {
    VectorND relative_position_AB = A_position - B_position;
    VectorND relative_velocity_AB = A_velocity - B_velocity;
    VectorND relative_acceleration_AB = A_acceleration - B_acceleration;
    double radii_sum = A_radius + B_radius;
    double radii_sum_sqrd = radii_sum * radii_sum;
    double distance_between_centers_sqrd = relative_position_AB.dot(relative_position_AB);
    if (distance_between_centers_sqrd <= radii_sum_sqrd) {
        intersection_time_start = 0.0;
        return true;
    }

    double a = 0.25 * relative_acceleration_AB.dot(relative_acceleration_AB);
    double b = relative_velocity_AB.dot(relative_acceleration_AB);
    double c = relative_position_AB.dot(relative_acceleration_AB) + relative_velocity_AB.dot(relative_velocity_AB);
    double d = 2.0 * relative_position_AB.dot(relative_velocity_AB);
    double e = distance_between_centers_sqrd - radii_sum_sqrd;

    // TODO SOLVE THIS QUARTIC FUNCTION
    return false;
}
