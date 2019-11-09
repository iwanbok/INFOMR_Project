// Based on Open3D implementation
#pragma once

#include "Open3D/Geometry/PointCloud.h"
#include <Eigen/Eigenvalues>
#include <tuple>
#include <vector>

/// Calculates the eigen values and vectors of the covariance matrix for the given points
std::tuple<Eigen::Vector3d, Eigen::Matrix3d>
ComputeEigenValuesAndVectors(const std::vector<Eigen::Vector3d> &points)
{
	// Compute covariance matrix
	open3d::geometry::PointCloud p(points);
	Eigen::Vector3d mean;
	Eigen::Matrix3d cov;
	std::tie(mean, cov) = p.ComputeMeanAndCovariance();

	// Use eigen's eigen vector solver
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
	Eigen::Vector3d evals = es.eigenvalues();
	Eigen::Matrix3d R = es.eigenvectors();
	// Normalize eigen vector(shouldn't be needed)
	R.col(0) /= R.col(0).norm();
	R.col(1) /= R.col(1).norm();
	R.col(2) /= R.col(2).norm();

	// Ensure order of eigen vectors from greatest to smallest.
	if (evals(1) > evals(0))
	{
		std::swap(evals(1), evals(0));
		R.col(1).swap(R.col(0));
	}
	if (evals(2) > evals(0))
	{
		std::swap(evals(2), evals(0));
		R.col(2).swap(R.col(0));
	}
	if (evals(2) > evals(1))
	{
		std::swap(evals(2), evals(1));
		R.col(2).swap(R.col(1));
	}
	return std::make_tuple(evals, R);
}