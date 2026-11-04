#include "geometry/linalg.hpp"

#include <algorithm>
#include <cmath>

namespace scand3d::geometry {
namespace {

Vec3 column(const Mat3& matrix, int index) {
    return {matrix[index], matrix[3 + index], matrix[6 + index]};
}

void setColumn(Mat3& matrix, int index, const Vec3& value) {
    matrix[index] = value[0];
    matrix[3 + index] = value[1];
    matrix[6 + index] = value[2];
}

Vec3 normalizedOrZero(const Vec3& value) {
    const double norm = std::sqrt(dot3(value, value));
    if (!(norm > 1e-15)) {
        return {0.0, 0.0, 0.0};
    }
    return {value[0] / norm, value[1] / norm, value[2] / norm};
}

bool isZero(const Vec3& value) {
    return value[0] == 0.0 && value[1] == 0.0 && value[2] == 0.0;
}

}  // namespace

Mat3 identity3() {
    return {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
}

Mat3 matmul3(const Mat3& a, const Mat3& b) {
    Mat3 out{};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) {
                sum += a[row * 3 + k] * b[k * 3 + col];
            }
            out[row * 3 + col] = sum;
        }
    }
    return out;
}

Mat3 transpose3(const Mat3& matrix) {
    return {matrix[0], matrix[3], matrix[6],
            matrix[1], matrix[4], matrix[7],
            matrix[2], matrix[5], matrix[8]};
}

Vec3 matvec3(const Mat3& matrix, const Vec3& vector) {
    return {matrix[0] * vector[0] + matrix[1] * vector[1] + matrix[2] * vector[2],
            matrix[3] * vector[0] + matrix[4] * vector[1] + matrix[5] * vector[2],
            matrix[6] * vector[0] + matrix[7] * vector[1] + matrix[8] * vector[2]};
}

Vec3 cross3(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

double dot3(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double determinant3(const Mat3& m) {
    return m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) +
           m[2] * (m[3] * m[7] - m[4] * m[6]);
}

Mat3 inverse3(const Mat3& m) {
    const double det = determinant3(m);
    if (!(std::fabs(det) > 1e-18)) {
        return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }
    const double inv = 1.0 / det;
    Mat3 out{};
    out[0] = (m[4] * m[8] - m[5] * m[7]) * inv;
    out[1] = (m[2] * m[7] - m[1] * m[8]) * inv;
    out[2] = (m[1] * m[5] - m[2] * m[4]) * inv;
    out[3] = (m[5] * m[6] - m[3] * m[8]) * inv;
    out[4] = (m[0] * m[8] - m[2] * m[6]) * inv;
    out[5] = (m[2] * m[3] - m[0] * m[5]) * inv;
    out[6] = (m[3] * m[7] - m[4] * m[6]) * inv;
    out[7] = (m[1] * m[6] - m[0] * m[7]) * inv;
    out[8] = (m[0] * m[4] - m[1] * m[3]) * inv;
    return out;
}

double frobeniusNorm3(const Mat3& matrix) {
    double sum = 0.0;
    for (const double value : matrix) {
        sum += value * value;
    }
    return std::sqrt(sum);
}

bool symmetricEigen(
    std::vector<double> a,
    int size,
    std::vector<double>& eigenvectors,
    std::vector<double>& eigenvalues) {
    const int n = size;
    if (n <= 0 || a.size() != static_cast<std::size_t>(n * n)) {
        return false;
    }

    eigenvectors.assign(static_cast<std::size_t>(n * n), 0.0);
    for (int i = 0; i < n; ++i) {
        eigenvectors[static_cast<std::size_t>(i * n + i)] = 1.0;
    }

    for (int sweep = 0; sweep < 40; ++sweep) {
        double off_diagonal = 0.0;
        double scale = 1.0;
        for (int p = 0; p < n; ++p) {
            const double diagonal = a[static_cast<std::size_t>(p * n + p)];
            scale += diagonal * diagonal;
            for (int q = p + 1; q < n; ++q) {
                const double value = a[static_cast<std::size_t>(p * n + q)];
                off_diagonal += value * value;
            }
        }
        if (off_diagonal < 1e-22 * scale) {
            break;
        }

        for (int p = 0; p < n - 1; ++p) {
            for (int q = p + 1; q < n; ++q) {
                const double apq = a[static_cast<std::size_t>(p * n + q)];
                if (std::fabs(apq) < 1e-20) {
                    continue;
                }
                const double app = a[static_cast<std::size_t>(p * n + p)];
                const double aqq = a[static_cast<std::size_t>(q * n + q)];
                const double theta = (aqq - app) / (2.0 * apq);
                const double sign = theta >= 0.0 ? 1.0 : -1.0;
                const double t = sign / (std::fabs(theta) + std::sqrt(theta * theta + 1.0));
                const double c = 1.0 / std::sqrt(t * t + 1.0);
                const double s = t * c;

                // A <- A J, then A <- J^T A: together that is the similarity transform, and the
                // same rotations accumulated on the identity give the eigenvectors.
                for (int k = 0; k < n; ++k) {
                    const auto kp = static_cast<std::size_t>(k * n + p);
                    const auto kq = static_cast<std::size_t>(k * n + q);
                    const double akp = a[kp];
                    const double akq = a[kq];
                    a[kp] = c * akp - s * akq;
                    a[kq] = s * akp + c * akq;
                }
                for (int k = 0; k < n; ++k) {
                    const auto pk = static_cast<std::size_t>(p * n + k);
                    const auto qk = static_cast<std::size_t>(q * n + k);
                    const double apk = a[pk];
                    const double aqk = a[qk];
                    a[pk] = c * apk - s * aqk;
                    a[qk] = s * apk + c * aqk;
                }
                for (int k = 0; k < n; ++k) {
                    const auto kp = static_cast<std::size_t>(k * n + p);
                    const auto kq = static_cast<std::size_t>(k * n + q);
                    const double vkp = eigenvectors[kp];
                    const double vkq = eigenvectors[kq];
                    eigenvectors[kp] = c * vkp - s * vkq;
                    eigenvectors[kq] = s * vkp + c * vkq;
                }
            }
        }
    }

    eigenvalues.assign(static_cast<std::size_t>(n), 0.0);
    for (int i = 0; i < n; ++i) {
        eigenvalues[static_cast<std::size_t>(i)] = a[static_cast<std::size_t>(i * n + i)];
    }
    return true;
}

bool symmetricEigen3(const Mat3& matrix, std::array<double, 3>& values, Mat3& eigenvectors) {
    std::vector<double> vectors;
    std::vector<double> eigenvalues;
    if (!symmetricEigen(std::vector<double>(matrix.begin(), matrix.end()), 3, vectors,
                        eigenvalues)) {
        return false;
    }
    for (int i = 0; i < 3; ++i) {
        values[static_cast<std::size_t>(i)] = eigenvalues[static_cast<std::size_t>(i)];
    }
    std::copy(vectors.begin(), vectors.end(), eigenvectors.begin());
    return true;
}

bool smallestEigenvector9(const std::array<double, 81>& symmetric, std::array<double, 9>& vector) {
    std::vector<double> vectors;
    std::vector<double> eigenvalues;
    if (!symmetricEigen(std::vector<double>(symmetric.begin(), symmetric.end()), 9, vectors,
                        eigenvalues)) {
        return false;
    }

    int smallest = 0;
    for (int i = 1; i < 9; ++i) {
        if (eigenvalues[static_cast<std::size_t>(i)] <
            eigenvalues[static_cast<std::size_t>(smallest)]) {
            smallest = i;
        }
    }

    double norm = 0.0;
    for (int row = 0; row < 9; ++row) {
        vector[static_cast<std::size_t>(row)] =
            vectors[static_cast<std::size_t>(row * 9 + smallest)];
        norm += vector[static_cast<std::size_t>(row)] * vector[static_cast<std::size_t>(row)];
    }
    norm = std::sqrt(norm);
    if (!(norm > 1e-15)) {
        return false;
    }
    for (double& value : vector) {
        value /= norm;
    }
    return true;
}

bool svd3(const Mat3& matrix, Mat3& u, std::array<double, 3>& singular_values, Mat3& v) {
    const Mat3 mtm = matmul3(transpose3(matrix), matrix);
    std::array<double, 3> eigenvalues{};
    Mat3 eigenvectors{};
    if (!symmetricEigen3(mtm, eigenvalues, eigenvectors)) {
        return false;
    }

    std::array<int, 3> order{0, 1, 2};
    std::sort(order.begin(), order.end(), [&eigenvalues](int a, int b) {
        return eigenvalues[static_cast<std::size_t>(a)] > eigenvalues[static_cast<std::size_t>(b)];
    });

    for (int j = 0; j < 3; ++j) {
        const int source = order[static_cast<std::size_t>(j)];
        singular_values[static_cast<std::size_t>(j)] =
            std::sqrt(std::max(0.0, eigenvalues[static_cast<std::size_t>(source)]));
        setColumn(v, j, column(eigenvectors, source));
    }

    std::array<Vec3, 3> columns{};
    for (int j = 0; j < 3; ++j) {
        const double sigma = singular_values[static_cast<std::size_t>(j)];
        if (sigma > 1e-12) {
            const Vec3 projected = matvec3(matrix, column(v, j));
            Vec3 scaled{};
            scaled[0] = projected[0] / sigma;
            scaled[1] = projected[1] / sigma;
            scaled[2] = projected[2] / sigma;
            columns[static_cast<std::size_t>(j)] = scaled;
        }
    }

    // Complete the basis for any (near-)zero singular value from the other two columns.
    if (singular_values[2] <= 1e-12) {
        columns[2] = normalizedOrZero(cross3(columns[0], columns[1]));
    }
    if (singular_values[1] <= 1e-12) {
        columns[1] = normalizedOrZero(cross3(columns[2], columns[0]));
    }
    if (singular_values[0] <= 1e-12) {
        columns[0] = normalizedOrZero(cross3(columns[1], columns[2]));
    }

    for (int j = 0; j < 3; ++j) {
        Vec3 candidate = normalizedOrZero(columns[static_cast<std::size_t>(j)]);
        if (isZero(candidate)) {
            // Degenerate input (e.g. a zero matrix): fall back to the standard basis.
            if (j == 0) {
                candidate = {1.0, 0.0, 0.0};
            } else if (j == 1) {
                candidate = {0.0, 1.0, 0.0};
            } else {
                candidate = {0.0, 0.0, 1.0};
            }
        }
        setColumn(u, j, candidate);
    }
    return true;
}

Mat3 composeFromSvd(const Mat3& u, const std::array<double, 3>& singular_values, const Mat3& v) {
    Mat3 out{};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) {
                sum += singular_values[static_cast<std::size_t>(k)] * u[row * 3 + k] *
                       v[col * 3 + k];
            }
            out[row * 3 + col] = sum;
        }
    }
    return out;
}

}  // namespace scand3d::geometry
