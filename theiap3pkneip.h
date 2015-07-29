#ifndef THEIAP3PKNEIOP
#define THEIAP3PKNEIOP

#include <vector>
#include <Eigen/Dense>

using namespace Eigen;

int SolveQuartic(const long double a, const long double b, const long double c,
                 const long double d, const long double e,
                 std::complex<long double>* roots);

int SolveQuarticReals(const long double a, const long double b,
                      const long double c, const long double d,
                      const long double e, const long double tolerance,
                      long double* roots);

int SolvePlaneRotation(const Vector3d normalized_image_points[3],
                       const Vector3d& intermediate_image_point,
                       const Vector3d& intermediate_world_point,
                       const double d_12,
                       long double cos_theta[4],
                       long double cot_alphas[4],
                       double* b);

void Backsubstitute(const Matrix3d& intermediate_world_frame,
                    const Matrix3d& intermediate_camera_frame,
                    const Vector3d& world_point_0,
                    const long double cos_theta,
                    const long double cot_alpha,
                    const double d_12,
                    const double b,
                    Vector3d* translation,
                    Matrix3d* rotation);

bool PoseFromThreePoints(const Vector2d feature_point[3],
                         const Vector3d points_3d[3],
                         std::vector<Matrix3d>* solution_rotations,
                         std::vector<Vector3d>* solution_translations);


#endif // THEIAP3PKNEIOP

