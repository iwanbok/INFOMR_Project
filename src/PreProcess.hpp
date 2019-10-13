#pragma once

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <filesystem>

#include "EigenVectors.hpp"

#include "Open3D/IO/TriangleMeshIO.h"

static bool endsWith(const std::string &str, const std::string &ending)
{
	if (str.length() < ending.length())
		return false;
	return str.compare(str.length() - ending.length(), ending.length(), ending) == 0;
}

/*
string split implementation by using delimiter as a character.
*/
std::vector<std::string> split(std::string strToSplit, char delimeter)
{
	std::stringstream ss(strToSplit);
	std::string item;
	std::vector<std::string> splittedStrings;
	while (getline(ss, item, delimeter))
	{
		splittedStrings.push_back(item);
	}
	return splittedStrings;
}

void simplify(const std::string &file, int targetF)
{
	auto mesh = open3d::io::CreateMeshFromFile(file);
	mesh->RemoveDuplicatedTriangles();
	mesh->RemoveDuplicatedVertices();
	mesh->RemoveDegenerateTriangles();
	mesh->RemoveUnreferencedVertices();
	auto mesh2 = mesh->SubdivideLoop(1);
	auto mesh_simple = mesh2->SimplifyQuadricDecimation(targetF);
	open3d::io::WriteTriangleMesh(file.substr(0, file.size() - 4) + "_cleaned.off", *mesh2, true,
								  true);
}

void WriteInfoFile(const std::string &file,
				   const std::shared_ptr<open3d::geometry::TriangleMesh> &mesh, double scaleFactor,
				   const Eigen::Matrix3d &evecs, const Eigen::Vector3d &evals)
{
	std::ofstream infoFile;
	infoFile.open(file + ".info");
	// mesh
	infoFile << file << std::endl;
	// class
	std::filesystem::path p(file);
	auto parent = p.parent_path();
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
	// Scale factor TODO
	infoFile << scaleFactor << std::endl;
	// Eigen Values
	infoFile << evals.transpose() << std::endl;
	// Major Eigen vector
	infoFile << evecs.col(0).transpose() << std::endl;
	// Medium Eigen vector
	infoFile << evecs.col(1).transpose() << std::endl;
	// Minor Eigen vector
	infoFile << evecs.col(2).transpose() << std::endl;

	infoFile.close();
}

void preProcessMeshDatabase(const std::vector<std::string> &files)
{
	float avgV = 0, avgF = 0;
	int minV = INT_MAX, maxV = 0, minF = INT_MAX, maxF = 0, numMeshes = 0;
	std::vector<std::string> outlier;

	for (auto &f : files)
		if ((endsWith(f, ".off") || endsWith(f, ".ply")) && !endsWith(f, "_scaled.off") &&
			find(files.begin(), files.end(), f.substr(0, f.size() - 4) + "_cleaned.off") ==
				files.end())
		{
			auto mesh = open3d::io::CreateMeshFromFile(f);

			auto scaleFactor = 0.5 / std::max((mesh->GetCenter() - mesh->GetMinBound()).maxCoeff(),
											  (mesh->GetMaxBound() - mesh->GetCenter()).maxCoeff());
			Eigen::Vector3d evals;
			Eigen::Matrix3d evecs;
			std::tie(evals, evecs) = ComputeEigenValuesAndVectors(mesh->vertices_);

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
				WriteInfoFile(f, mesh, scaleFactor, evecs, evals);			

			mesh->Translate(-mesh->GetCenter());
			Eigen::Affine3d transform;
			transform.linear() << evecs.transpose();
			mesh->Transform(transform.matrix());

			open3d::io::WriteTriangleMesh(f.substr(0, f.size() - 4) + "_scaled.off", *mesh, true,
										  true);
			numMeshes++;
		}

	avgV /= numMeshes;
	avgF /= numMeshes;
	printf("# of Vertices:\n\tAvg: %.2f\n\tMin: %i\n\tMax:%i\n# of Faces:\n\tAvg: %.2f\n\tMin: "
		   "%i\n\tMax: %i\n",
		   avgV, minV, maxV, avgF, minF, maxF);
	for (auto &f : outlier)
		simplify(f, (int)avgF);
}