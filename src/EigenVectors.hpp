// Based on Open3D implementation
#pragma once

#include <Eigen/Eigenvalues>

Eigen::Matrix3d ComputeEigenVectors(const Eigen::MatrixX3d &points)
{
	Eigen::Matrix3d covariance;
	Eigen::Matrix<double, 9, 1> cumulants;
	cumulants.setZero();
	for (size_t i = 0; i < points.rows(); i++)
	{
		const Eigen::Vector3d &point = points.row(i);
		cumulants(0) += point(0);
		cumulants(1) += point(1);
		cumulants(2) += point(2);
		cumulants(3) += point(0) * point(0);
		cumulants(4) += point(0) * point(1);
		cumulants(5) += point(0) * point(2);
		cumulants(6) += point(1) * point(1);
		cumulants(7) += point(1) * point(2);
		cumulants(8) += point(2) * point(2);
	}
	cumulants /= (double)points.rows();
	covariance(0, 0) = cumulants(3) - cumulants(0) * cumulants(0);
	covariance(1, 1) = cumulants(6) - cumulants(1) * cumulants(1);
	covariance(2, 2) = cumulants(8) - cumulants(2) * cumulants(2);
	covariance(0, 1) = cumulants(4) - cumulants(0) * cumulants(1);
	covariance(1, 0) = covariance(0, 1);
	covariance(0, 2) = cumulants(5) - cumulants(0) * cumulants(2);
	covariance(2, 0) = covariance(0, 2);
	covariance(1, 2) = cumulants(7) - cumulants(1) * cumulants(2);
	covariance(2, 1) = covariance(1, 2);

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
	solver.compute(covariance, Eigen::ComputeEigenvectors);
    return solver.eigenvectors();
}