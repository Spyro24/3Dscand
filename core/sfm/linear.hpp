#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

namespace scand3d::sfm::linear {

/// Solves a dense square system by Gaussian elimination with partial pivoting.
/// Returns false when the matrix is numerically singular or the solution is not finite.
inline bool solveDense(
    std::vector<double> matrix,
    int size,
    std::vector<double> rhs,
    std::vector<double>& solution) {
    if (size <= 0 ||
        matrix.size() != static_cast<std::size_t>(size) * static_cast<std::size_t>(size) ||
        rhs.size() != static_cast<std::size_t>(size)) {
        return false;
    }

    solution.assign(static_cast<std::size_t>(size), 0.0);
    const auto n = static_cast<std::size_t>(size);

    for (int column = 0; column < size; ++column) {
        int pivot = column;
        double best = std::fabs(matrix[static_cast<std::size_t>(column) * n +
                                       static_cast<std::size_t>(column)]);
        for (int row = column + 1; row < size; ++row) {
            const double value = std::fabs(matrix[static_cast<std::size_t>(row) * n +
                                                 static_cast<std::size_t>(column)]);
            if (value > best) {
                best = value;
                pivot = row;
            }
        }
        if (!(best > 1e-14)) {
            return false;
        }

        if (pivot != column) {
            for (std::size_t k = 0; k < n; ++k) {
                std::swap(matrix[static_cast<std::size_t>(column) * n + k],
                          matrix[static_cast<std::size_t>(pivot) * n + k]);
            }
            std::swap(rhs[static_cast<std::size_t>(column)],
                      rhs[static_cast<std::size_t>(pivot)]);
        }

        const double diagonal = matrix[static_cast<std::size_t>(column) * n +
                                       static_cast<std::size_t>(column)];
        for (int row = column + 1; row < size; ++row) {
            const double factor =
                matrix[static_cast<std::size_t>(row) * n + static_cast<std::size_t>(column)] /
                diagonal;
            if (factor == 0.0) {
                continue;
            }
            for (int k = column; k < size; ++k) {
                matrix[static_cast<std::size_t>(row) * n + static_cast<std::size_t>(k)] -=
                    factor * matrix[static_cast<std::size_t>(column) * n +
                                    static_cast<std::size_t>(k)];
            }
            rhs[static_cast<std::size_t>(row)] -= factor * rhs[static_cast<std::size_t>(column)];
        }
    }

    for (int row = size - 1; row >= 0; --row) {
        double sum = rhs[static_cast<std::size_t>(row)];
        for (int k = row + 1; k < size; ++k) {
            sum -= matrix[static_cast<std::size_t>(row) * n + static_cast<std::size_t>(k)] *
                   solution[static_cast<std::size_t>(k)];
        }
        const double diagonal =
            matrix[static_cast<std::size_t>(row) * n + static_cast<std::size_t>(row)];
        solution[static_cast<std::size_t>(row)] = sum / diagonal;
        if (!std::isfinite(solution[static_cast<std::size_t>(row)])) {
            return false;
        }
    }
    return true;
}

}  // namespace scand3d::sfm::linear
