#pragma once
class VectorND;

bool NSphereCollisionCheckStatic(const VectorND& A_position,
                                 const double A_radius,
                                 const VectorND& B_position,
                                 const double B_radius);

bool NSphereCollisionCheckDynamic(const VectorND& A_position,
                                  const VectorND& A_velocity,
                                  const double A_radius,
                                  const VectorND& B_position,
                                  const VectorND& B_velocity,
                                  const double B_radius);

bool timeUntillNSphereCollision(const VectorND& A_position,
                                const VectorND& A_velocity,
                                const double A_radius,
                                const VectorND& B_position,
                                const VectorND& B_velocity,
                                const double B_radius,
                                double& intersection_time_start);

bool timeUntillNSphereCollision(const VectorND& A_position,
                                const VectorND& A_velocity,
                                const double A_radius,
                                const VectorND& B_position,
                                const VectorND& B_velocity,
                                const double B_radius,
                                double& intersection_time_start,
                                double& intersection_time_end);

bool timeUntillNSphereCollisionAccurate(const VectorND& A_position,
                                        const VectorND& A_velocity,
                                        const VectorND& A_acceleration,
                                        const double A_radius,
                                        const VectorND& B_position,
                                        const VectorND& B_velocity,
                                        const VectorND& B_acceleration,
                                        const double B_radius,
                                        double& intersection_time_start);
