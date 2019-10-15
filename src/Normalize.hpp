#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "EigenVectors.hpp"
#include "Utils.hpp"

void Normalize(const std::shared_ptr<open3d::geometry::TriangleMesh> &mesh)
{
	Eigen::Vector3d evals;
	Eigen::Matrix3d evecs;
	std::tie(evals, evecs) = ComputeEigenValuesAndVectors(mesh->vertices_);

	mesh->Translate(-mesh->GetCenter());
	Eigen::Affine3d transform;
	transform.linear() << evecs.transpose();
	mesh->Transform(transform.matrix());
	mesh->Scale(1 / (mesh->GetMaxBound() - mesh->GetMinBound()).maxCoeff());
}

void NormalizeMeshDataBase(const std::vector<std::filesystem::path> &files)
{
	CalcMeshStatistics(files);
	for (auto &f : files)
	{
		auto mesh = open3d::io::CreateMeshFromFile(f.string());

		Normalize(mesh);

		open3d::io::WriteTriangleMesh(replaceDir(f, NORMALIZED_DIR), *mesh, true, true);
	}
}