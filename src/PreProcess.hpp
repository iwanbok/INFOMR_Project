#pragma once

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "Open3D/IO/TriangleMeshIO.h"

#include "EigenVectors.hpp"
#include "Utils.hpp"

void Simplify(const std::filesystem::path &file, int targetF)
{
	auto mesh = open3d::io::CreateMeshFromFile(file.string());
	mesh->RemoveDuplicatedTriangles();
	mesh->RemoveDuplicatedVertices();
	mesh->RemoveDegenerateTriangles();
	mesh->RemoveUnreferencedVertices();
	auto mesh2 = mesh->SubdivideLoop(1);
	auto mesh_simple = mesh2->SimplifyQuadricDecimation(targetF);
	open3d::io::WriteTriangleMesh(replaceDir(file, PREPROCESSED_DIR), *mesh_simple, true, true);
}

void WriteInfoFile(const std::filesystem::path &file,
				   const std::shared_ptr<open3d::geometry::TriangleMesh> &mesh)
{
	std::ofstream infoFile;
	infoFile.open(replaceDir(file, INFO_DIR) / ".info");
	// mesh
	infoFile << file << std::endl;
	// class
	auto parent = file.parent_path();
	infoFile << parent.string().substr(parent.parent_path().string().size() + 1) << std::endl;
	// # vertices
	infoFile << mesh->vertices_.size() << std::endl;
	// # faces
	infoFile << mesh->triangles_.size() << std::endl;
	// Minimal corner
	infoFile << mesh->GetMinBound().transpose() << std::endl;
	// Maximal corner
	infoFile << mesh->GetMaxBound().transpose() << std::endl;
	// Barycenter
	infoFile << mesh->GetCenter().transpose() << std::endl;

	infoFile.close();
}

void PreProcessMeshDatabase(const std::vector<std::filesystem::path> &files)
{
	float avgV = 0, avgF = 0;
	int minV = INT_MAX, maxV = 0, minF = INT_MAX, maxF = 0, numMeshes = 0;
	std::vector<std::filesystem::path> outlier;

	for (auto &f : files)
		if (f.extension().string().compare(".off") == 0 ||
			f.extension().string().compare(".ply") == 0)
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

			if (f_count > 30000 || f_count < 10000)
				outlier.push_back(f);
			else
			{
				WriteInfoFile(f, mesh);
				open3d::io::WriteTriangleMesh(replaceDir(f, PREPROCESSED_DIR), *mesh, true, true);
			}
			numMeshes++;
		}

	avgV /= numMeshes;
	avgF /= numMeshes;
	printf("# of Vertices:\n\tAvg: %.2f\n\tMin: %i\n\tMax:%i\n# of Faces:\n\tAvg: %.2f\n\tMin: "
		   "%i\n\tMax: %i\n",
		   avgV, minV, maxV, avgF, minF, maxF);
	for (auto &f : outlier)
		Simplify(f, (int)avgF);
}