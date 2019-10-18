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

void PreProcess(std::shared_ptr<open3d::geometry::TriangleMesh> &mesh)
{
	mesh->RemoveDuplicatedTriangles();
	mesh->RemoveDuplicatedVertices();
	mesh->RemoveDegenerateTriangles();
	mesh->RemoveUnreferencedVertices();
	if (mesh->triangles_.size() < lowerBoundF)
		while (mesh->triangles_.size() < targetF)
			mesh = mesh->SubdivideLoop(1);
	if (mesh->triangles_.size() > upperBoundF)
		mesh = mesh->SimplifyQuadricDecimation(targetF);
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
	CalcMeshStatistics(files);
	for (auto &f : files)
		if (std::find(exts.begin(), exts.end(), f.extension().string()) != exts.end())
		{
			auto mesh = open3d::io::CreateMeshFromFile(f.string());
			PreProcess(mesh);
			WriteInfoFile(f, mesh);
			open3d::io::WriteTriangleMesh(replaceDir(f, PREPROCESSED_DIR).string(), *mesh, true, true);
		}
}