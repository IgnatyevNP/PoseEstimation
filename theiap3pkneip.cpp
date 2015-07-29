#include "theiap3pkneip.h"

int SolveQuartic(const long double a, const long double b, const long double c,
                 const long double d, const long double e,
                 std::complex<long double>* roots) {
    const long double a_pw2 = a * a;
    const long double b_pw2 = b * b;
    const long double a_pw3 = a_pw2 * a;
    const long double b_pw3 = b_pw2 * b;
    const long double a_pw4 = a_pw3 * a;
    const long double b_pw4 = b_pw3 * b;

    const long double alpha = -3.0l * b_pw2 / (8.0l * a_pw2) + c / a;
    const long double beta =
            b_pw3 / (8.0l * a_pw3) - b * c / (2.0l * a_pw2) + d / a;
    const long double gamma =
            -3.0l * b_pw4 / (256.0l * a_pw4) + b_pw2 * c / (16.0l * a_pw3) -
            b * d / (4.0l * a_pw2) + e / a;

    const long double alpha_pw2 = alpha * alpha;
    const long double alpha_pw3 = alpha_pw2 * alpha;

    const std::complex<long double> P(-alpha_pw2 / 12.0l - gamma, 0);
    const std::complex<long double> Q(
                -alpha_pw3 / 108.0l + alpha * gamma / 3.0l - std::pow(beta, 2.0l) / 8.0l,
                0);
    const std::complex<long double> R =
            -Q / 2.0l +
            std::sqrt(std::pow(Q, 2.0) / 4.0l + std::pow(P, 3.0l) / 27.0l);

    const std::complex<long double> U = std::pow(R, (1.0l / 3.0l));
    std::complex<long double> y;

    const long double kEpsilon = 1e-8;
    if (fabs(U.real()) < kEpsilon) {
        y = -5.0l * alpha / 6.0l - std::pow(Q, (1.0l / 3.0l));
    } else {
        y = -5.0l * alpha / 6.0l - P / (3.0l * U) + U;
    }

    const std::complex<long double> w = std::sqrt(alpha + 2.0l * y);

    roots[0] =
            -b / (4.0l * a) +
            0.5l * (w + std::sqrt(-(3.0l * alpha + 2.0l * y + 2.0l * beta / w)));
    roots[1] =
            -b / (4.0l * a) +
            0.5l * (w - std::sqrt(-(3.0l * alpha + 2.0l * y + 2.0l * beta / w)));
    roots[2] =
            -b / (4.0l * a) +
            0.5l * (-w + std::sqrt(-(3.0l * alpha + 2.0l * y - 2.0l * beta / w)));
    roots[3] =
            -b / (4.0l * a) +
            0.5l * (-w - std::sqrt(-(3.0l * alpha + 2.0l * y - 2.0l * beta / w)));

    return 4;
}

int SolveQuarticReals(const long double a, const long double b,
                      const long double c, const long double d,
                      const long double e, const long double tolerance,
                      long double* roots) {
  std::complex<long double> complex_roots[4];
  int num_complex_solutions = SolveQuartic(a, b, c, d, e, complex_roots);
  int num_real_solutions = 0;
  for (int i = 0; i < num_complex_solutions; i++) {
    if (fabs(complex_roots[i].imag()) < tolerance) {
      roots[num_real_solutions++] = complex_roots[i].real();
    }
  }
  return num_real_solutions;
}

// Solves for cos(theta) that will describe the rotation of the plane from
// intermediate world frame to intermediate camera frame. The method returns the
// roots of a quartic (i.e. solutions to cos(alpha) ) and several factors that
// are needed for back-substitution.
int SolvePlaneRotation(const Vector3d normalized_image_points[3],
                       const Vector3d& intermediate_image_point,
                       const Vector3d& intermediate_world_point,
                       const double d_12,
                       long double cos_theta[4],
                       long double cot_alphas[4],
                       double* b) {
    // Calculate these parameters ahead of time for reuse and
    // readability. Notation for these variables is consistent with the notation
    // from the paper.
    const long double f_1 =
            intermediate_image_point[0] / intermediate_image_point[2];
    const long double f_2 =
            intermediate_image_point[1] / intermediate_image_point[2];
    const long double p_1 = intermediate_world_point[0];
    const long double p_2 = intermediate_world_point[1];
    const long double cos_beta =
            normalized_image_points[0].dot(normalized_image_points[1]);
    *b = 1.0 / (1.0 - cos_beta * cos_beta) - 1.0;

    if (cos_beta < 0) {
        *b = -sqrt(*b);
    } else {
        *b = sqrt(*b);
    }

    // Definition of temporary variables for readability in the coefficients
    // calculation.
    const long double f_1_pw2 = f_1 * f_1;
    const long double f_2_pw2 = f_2 * f_2;
    const long double p_1_pw2 = p_1 * p_1;
    const long double p_1_pw3 = p_1_pw2 * p_1;
    const long double p_1_pw4 = p_1_pw3 * p_1;
    const long double p_2_pw2 = p_2 * p_2;
    const long double p_2_pw3 = p_2_pw2 * p_2;
    const long double p_2_pw4 = p_2_pw3 * p_2;
    const long double d_12_pw2 = d_12 * d_12;
    const long double b_pw2 = (*b) * (*b);

    // Computation of coefficients of 4th degree polynomial.
    Eigen::Matrix<long double, 5, 1> coefficients;
    coefficients[0] = -f_2_pw2 * p_2_pw4 - p_2_pw4 * f_1_pw2 - p_2_pw4;
    coefficients[1] =
            2.0 * p_2_pw3 * d_12 * (*b) + 2.0 * f_2_pw2 * p_2_pw3 * d_12 * (*b) -
            2.0 * f_2 * p_2_pw3 * f_1 * d_12;
    coefficients[2] =
            -f_2_pw2 * p_2_pw2 * p_1_pw2 - f_2_pw2 * p_2_pw2 * d_12_pw2 * b_pw2 -
            f_2_pw2 * p_2_pw2 * d_12_pw2 + f_2_pw2 * p_2_pw4 + p_2_pw4 * f_1_pw2 +
            2.0 * p_1 * p_2_pw2 * d_12 +
            2.0 * f_1 * f_2 * p_1 * p_2_pw2 * d_12 * (*b) -
            p_2_pw2 * p_1_pw2 * f_1_pw2 + 2.0 * p_1 * p_2_pw2 * f_2_pw2 * d_12 -
            p_2_pw2 * d_12_pw2 * b_pw2 - 2.0 * p_1_pw2 * p_2_pw2;
    coefficients[3] =
            2.0 * p_1_pw2 * p_2 * d_12 * (*b) + 2.0 * f_2 * p_2_pw3 * f_1 * d_12 -
            2.0 * f_2_pw2 * p_2_pw3 * d_12 * (*b) - 2.0 * p_1 * p_2 * d_12_pw2 * (*b);
    coefficients[4] =
            -2 * f_2 * p_2_pw2 * f_1 * p_1 * d_12 * (*b) +
            f_2_pw2 * p_2_pw2 * d_12_pw2 + 2.0 * p_1_pw3 * d_12 - p_1_pw2 * d_12_pw2 +
            f_2_pw2 * p_2_pw2 * p_1_pw2 - p_1_pw4 -
            2.0 * f_2_pw2 * p_2_pw2 * p_1 * d_12 + p_2_pw2 * f_1_pw2 * p_1_pw2 +
            f_2_pw2 * p_2_pw2 * d_12_pw2 * b_pw2;

    // Computation of roots.
    const long double kEpsilon = 1e-6;
    const int num_solutions = SolveQuarticReals(
                coefficients[0], coefficients[1], coefficients[2], coefficients[3],
                coefficients[4], kEpsilon, cos_theta);



    // Calculate cot(alpha) needed for back-substitution.
    for (int i = 0; i < num_solutions; i++) {
        cot_alphas[i] = (-f_1 * p_1 / f_2 - cos_theta[i] * p_2 + d_12 * (*b)) /
                (-f_1 * cos_theta[i] * p_2 / f_2 + p_1 - d_12);
    }

    return num_solutions;
}

void Backsubstitute(const Matrix3d& intermediate_world_frame,
                    const Matrix3d& intermediate_camera_frame,
                    const Vector3d& world_point_0,
                    const long double cos_theta,
                    const long double cot_alpha,
                    const double d_12,
                    const double b,
                    Vector3d* translation,
                    Matrix3d* rotation) {
    const double sin_theta = sqrt(1.0 - cos_theta * cos_theta);
    const double sin_alpha = sqrt(1.0 / (cot_alpha * cot_alpha + 1.0));
    double cos_alpha = sqrt(1.0 - sin_alpha * sin_alpha);

    if (cot_alpha < 0) {
        cos_alpha = -cos_alpha;
    }

    // Get the camera position in the intermediate world frame
    // coordinates. (Eq. 5 from the paper).
    const Vector3d c_nu(
                d_12 * cos_alpha * (sin_alpha * b + cos_alpha),
                cos_theta * d_12 * sin_alpha * (sin_alpha * b + cos_alpha),
                sin_theta * d_12 * sin_alpha * (sin_alpha * b + cos_alpha));

    // Transform c_nu into world coordinates. Use a Map to put the solution
    // directly into the output.
    *translation = world_point_0 + intermediate_world_frame.transpose() * c_nu;

    // Construct the transformation from the intermediate world frame to the
    // intermediate camera frame.
    Matrix3d intermediate_world_to_camera_rotation;
    intermediate_world_to_camera_rotation <<
                                             -cos_alpha, -sin_alpha * cos_theta, -sin_alpha * sin_theta,
            sin_alpha, -cos_alpha * cos_theta, -cos_alpha * sin_theta,
            0, -sin_theta, cos_theta;

    // Construct the rotation matrix.
    *rotation = (intermediate_world_frame.transpose() *
                 intermediate_world_to_camera_rotation.transpose() *
                 intermediate_camera_frame).transpose();

    // Adjust translation to account for rotation.
    *translation = -(*rotation) * (*translation);
}

bool PoseFromThreePoints(const Vector2d feature_point[3],
                         const Vector3d points_3d[3],
                         std::vector<Matrix3d>* solution_rotations,
                         std::vector<Vector3d>* solution_translations) {
    Vector3d normalized_image_points[3];
    // Store points_3d in world_points for ease of use. NOTE: we cannot use a
    // const ref or a Map because the world_points entries may be swapped later.
    Vector3d world_points[3];
    for (int i = 0; i < 3; ++i) {
        normalized_image_points[i] = feature_point[i].homogeneous().normalized();
        world_points[i] = points_3d[i];
    }

    // If the points are collinear, there are no possible solutions.
    double kTolerance = 1e-6;
    Vector3d world_1_0 = world_points[1] - world_points[0];
    Vector3d world_2_0 = world_points[2] - world_points[0];
    if (world_1_0.cross(world_2_0).squaredNorm() < kTolerance)
        return false;

    // Create intermediate camera frame such that the x axis is in the direction
    // of one of the normalized image points, and the origin is the same as the
    // absolute camera frame. This is a rotation defined as the transformation:
    // T = [tx, ty, tz] where tx = f0, tz = (f0 x f1) / ||f0 x f1||, and
    // ty = tx x tz and f0, f1, f2 are the normalized image points.
    Matrix3d intermediate_camera_frame;
    intermediate_camera_frame.row(0) = normalized_image_points[0];
    intermediate_camera_frame.row(2) =
            normalized_image_points[0].cross(normalized_image_points[1]).normalized();
    intermediate_camera_frame.row(1) =
            intermediate_camera_frame.row(2).cross(intermediate_camera_frame.row(0));

    // Project the third world point into the intermediate camera frame.
    Vector3d intermediate_image_point =
            intermediate_camera_frame * normalized_image_points[2];

    // Enforce that the intermediate_image_point is in front of the intermediate
    // camera frame. If the point is behind the camera frame, recalculate the
    // intermediate camera frame by swapping which feature we align the x axis to.
    if (intermediate_image_point[2] > 0) {
        std::swap(normalized_image_points[0], normalized_image_points[1]);

        intermediate_camera_frame.row(0) = normalized_image_points[0];
        intermediate_camera_frame.row(2) = normalized_image_points[0]
                .cross(normalized_image_points[1]).normalized();
        intermediate_camera_frame.row(1) = intermediate_camera_frame.row(2)
                .cross(intermediate_camera_frame.row(0));

        intermediate_image_point =
                intermediate_camera_frame * normalized_image_points[2];

        std::swap(world_points[0], world_points[1]);
        world_1_0 = world_points[1] - world_points[0];
        world_2_0 = world_points[2] - world_points[0];
    }

    // Create the intermediate world frame transformation that has the
    // origin at world_points[0] and the x-axis in the direction of
    // world_points[1]. This is defined by the transformation: N = [nx, ny, nz]
    // where nx = (p1 - p0) / ||p1 - p0||
    // nz = nx x (p2 - p0) / || nx x (p2 -p0) || and ny = nz x nx
    // Where p0, p1, p2 are the world points.
    Matrix3d intermediate_world_frame;
    intermediate_world_frame.row(0) = world_1_0.normalized();
    intermediate_world_frame.row(2) =
            intermediate_world_frame.row(0).cross(world_2_0).normalized();
    intermediate_world_frame.row(1) =
            intermediate_world_frame.row(2).cross(intermediate_world_frame.row(0));

    // Transform world_point[2] to the intermediate world frame coordinates.
    Vector3d intermediate_world_point = intermediate_world_frame * world_2_0;

    // Distance from world_points[1] to the intermediate world frame origin.
    double d_12 = world_1_0.norm();

    // Solve for the cos(theta) that will give us the transformation from
    // intermediate world frame to intermediate camera frame. We also get the
    // cot(alpha) for each solution necessary for back-substitution.
    long double cos_theta[4];
    long double cot_alphas[4];
    double b;
    const int num_solutions = SolvePlaneRotation(
                normalized_image_points, intermediate_image_point,
                intermediate_world_point, d_12, cos_theta, cot_alphas, &b);

    // Backsubstitution of each solution
    solution_translations->resize(num_solutions);
    solution_rotations->resize(num_solutions);
    for (int i = 0; i < num_solutions; i++) {
        Backsubstitute(intermediate_world_frame,
                       intermediate_camera_frame,
                       world_points[0],
                       cos_theta[i],
                       cot_alphas[i],
                       d_12,
                       b,
                       &solution_translations->at(i),
                       &solution_rotations->at(i));
    }

    return num_solutions > 0;
}
