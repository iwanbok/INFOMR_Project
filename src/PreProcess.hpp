#pragma once

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "Open3D/IO/TriangleMeshIO.h"

#include "defines.h"
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

/// Preprocesses the entire mesh database
void PreProcessMeshDatabase(const std::vector<std::filesystem::path> &files)
{
	CalcMeshStatistics(files);
	for (auto &f : files)
		if (std::find(exts.begin(), exts.end(), f.extension().string()) != exts.end())
		{
			auto mesh = open3d::io::CreateMeshFromFile(f.string());
			PreProcess(mesh);
			open3d::io::WriteTriangleMesh(replaceDir(f, PREPROCESSED_DIR).string(), *mesh, true,
										  true);
		}
}