#pragma once

#include <array>
#include <vector>

namespace scand3d::geometry {

using Vec2 = std::array<double, 2>;
using Vec3 = std::array<double, 3>;
using Mat3 = std::array<double, 9>;   // row-major
using Mat4 = std::array<double, 16>;  // row-major

Mat3 identity3();
Mat3 matmul3(const Mat3& a, const Mat3& b);
Mat3 transpose3(const Mat3& matrix);
Vec3 matvec3(const Mat3& matrix, const Vec3& vector);
Vec3 cross3(const Vec3& a, const Vec3& b);
double dot3(const Vec3& a, const Vec3& b);
double determinant3(const Mat3& matrix);

/// Adjugate inverse; returns a zero matrix for a singular input, which callers must check.
Mat3 inverse3(const Mat3& matrix);

/// Frobenius norm, used to normalize a matrix that is only defined up to scale.
double frobeniusNorm3(const Mat3& matrix);

/// Eigen-decomposition of a symmetric matrix by cyclic Jacobi rotations. `values` holds the
/// eigenvalues; `vectors` has the matching eigenvectors as *columns* (row-major, n x n).
bool symmetricEigen(
    std::vector<double> symmetric_row_major,
    int size,
    std::vector<double>& vectors,
    std::vector<double>& values);

/// Convenience wrappers for the 3x3 and 9x9 cases the two-view estimators need.
bool symmetricEigen3(const Mat3& matrix, std::array<double, 3>& values, Mat3& eigenvectors);
bool smallestEigenvector9(const std::array<double, 81>& symmetric, std::array<double, 9>& vector);

/// SVD of a 3x3 matrix: M = U diag(s) V^T, singular values sorted descending.
bool svd3(const Mat3& matrix, Mat3& u, std::array<double, 3>& singular_values, Mat3& v);

/// Rebuilds U diag(s) V^T, used to enforce rank 2 or equal singular values.
Mat3 composeFromSvd(const Mat3& u, const std::array<double, 3>& singular_values, const Mat3& v);

}  // namespace scand3d::geometry
