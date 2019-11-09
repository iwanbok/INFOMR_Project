#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "EigenVectors.hpp"
#include "Utils.hpp"

/// Normalizes a single mesh in place
void Normalize(const std::shared_ptr<open3d::geometry::TriangleMesh> &mesh)
{
	// 1. Translate to barycenter
	mesh->Translate(-mesh->GetCenter());

	// 2. Rotate to align with eigenvectors
	Eigen::Affine3d orient;
	orient.setIdentity();
	Eigen::Vector3d evals;
	Eigen::Matrix3d evecs;
	std::tie(evals, evecs) = ComputeEigenValuesAndVectors(mesh->vertices_);
	orient.linear() << evecs.transpose();
	mesh->Transform(orient.matrix());

	// 3. Flipping via moment test
	double f_x = 0, f_y = 0, f_z = 0;
	for (const auto &t : mesh->triangles_)
	{
		Eigen::Vector3d centriod =
			(1.0 / 3.0) * (mesh->vertices_[t(0)] + mesh->vertices_[t(1)] + mesh->vertices_[t(2)]);
		f_x += sgn(centriod(0)) * centriod(0) * centriod(0);
		f_y += sgn(centriod(1)) * centriod(1) * centriod(1);
		f_z += sgn(centriod(2)) * centriod(2) * centriod(2);
	}

	Eigen::Affine3d flipping;
	flipping.setIdentity();
	flipping.linear().diagonal() << sgn(f_x), sgn(f_y), sgn(f_z);
	mesh->Transform(flipping.matrix());

	// 4. Scale to unit cube
	mesh->Scale(1.0 / (mesh->GetMaxBound() - mesh->GetMinBound()).maxCoeff());
}

/// Normalizes the entire mesh database
void NormalizeMeshDataBase(const std::vector<std::filesystem::path> &files)
{
	CalcMeshStatistics(files);
	for (auto &f : files)
	{
		auto mesh = open3d::io::CreateMeshFromFile(f.string());

		Normalize(mesh);

		open3d::io::WriteTriangleMesh(replaceDir(f, NORMALIZED_DIR).string(), *mesh, true, true);
	}
}