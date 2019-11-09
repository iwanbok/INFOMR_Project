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

/// Preprossess a single mesh in place
void PreProcess(std::shared_ptr<open3d::geometry::TriangleMesh> &mesh)
{
	// Basic cleaning
	mesh->RemoveDuplicatedTriangles();
	mesh->RemoveDuplicatedVertices();
	mesh->RemoveDegenerateTriangles();
	mesh->RemoveUnreferencedVertices();

	// Upsample the mesh if needed(can go over upperBoundF)
	if (mesh->triangles_.size() < lowerBoundF)
		while (mesh->triangles_.size() < targetF)
			mesh = mesh->SubdivideMidpoint(1);
	// Downsample mesh if needed(CANNOT go below targetF)
	if (mesh->triangles_.size() > upperBoundF)
		mesh = mesh->SimplifyQuadricDecimation(targetF);
}

/// Writes an info file with basic mesh information(info no longer used).
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

/// Preprocesses the entire mesh database
void PreProcessMeshDatabase(const std::vector<std::filesystem::path> &files)
{
	CalcMeshStatistics(files);
	for (auto &f : files)
		if (std::find(exts.begin(), exts.end(), f.extension().string()) != exts.end())
		{
			auto mesh = open3d::io::CreateMeshFromFile(f.string());
			PreProcess(mesh);
			WriteInfoFile(f, mesh);
			open3d::io::WriteTriangleMesh(replaceDir(f, PREPROCESSED_DIR).string(), *mesh, true,
										  true);
		}
}