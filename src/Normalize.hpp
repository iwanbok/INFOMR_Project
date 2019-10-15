#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "EigenVectors.hpp"
#include "Utils.hpp"

void NormalizeMeshDataBase(const std::vector<std::filesystem::path> &files)
{
	float avgV = 0, avgF = 0;
	int minV = INT_MAX, maxV = 0, minF = INT_MAX, maxF = 0;
	for (auto &f : files)
	{
		auto mesh = open3d::io::CreateMeshFromFile(f.string());

		int v_count = mesh->vertices_.size();
		avgV += v_count;
		minV = std::min(minV, v_count);
		maxV = std::max(maxV, v_count);

		int f_count = mesh->triangles_.size();
		avgF += f_count;
		minF = std::min(minF, f_count);
		maxF = std::max(maxF, f_count);

		auto scaleFactor = 0.5 / std::max((mesh->GetCenter() - mesh->GetMinBound()).maxCoeff(),
										  (mesh->GetMaxBound() - mesh->GetCenter()).maxCoeff());

		Eigen::Vector3d evals;
		Eigen::Matrix3d evecs;
		std::tie(evals, evecs) = ComputeEigenValuesAndVectors(mesh->vertices_);

		mesh->Translate(-mesh->GetCenter());
		Eigen::Affine3d transform;
		transform.linear() << evecs.transpose();
		mesh->Transform(transform.matrix());

		open3d::io::WriteTriangleMesh(replaceDir(f, NORMALIZED_DIR), *mesh, true, true);
	}

	avgV /= files.size();
	avgF /= files.size();
	printf("# of Vertices:\n\tAvg: %.2f\n\tMin: %i\n\tMax:%i\n# of Faces:\n\tAvg: %.2f\n\tMin: "
		   "%i\n\tMax: %i\n",
		   avgV, minV, maxV, avgF, minF, maxF);
}