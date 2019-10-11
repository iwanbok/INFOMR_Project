#pragma once

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

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

void simplify(std::string file, int targetF)
{
	auto mesh = open3d::io::CreateMeshFromFile(file);
	mesh->RemoveDuplicatedTriangles();
	mesh->RemoveDuplicatedVertices();
	mesh->RemoveDegenerateTriangles();
	mesh->RemoveUnreferencedVertices();
	auto mesh2 = mesh->SubdivideLoop(1);
	auto mesh_simple = mesh2->SimplifyQuadricDecimation(targetF);
	open3d::io::WriteTriangleMesh(file.substr(0, file.size() - 4) + "_cleaned.off", *mesh2, true, true);
}

void preProcessMeshDatabase(const std::vector<std::string> &files)
{
	float avgV = 0, avgF = 0;
	int minV = INT_MAX, maxV = 0, minF = INT_MAX, maxF = 0, numMeshes = 0;
	std::vector<std::string> outlier;
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;

	for (auto &f : files)
		if ((endsWith(f, ".off") || endsWith(f, ".ply")) &&
			find(files.begin(), files.end(), f.substr(0, f.size() - 4) + "_cleaned.off") ==
				files.end())
		{
			auto splitted = split(f, '\\');
			auto mesh = open3d::io::CreateMeshFromFile(f);
			V = Eigen::Map<Eigen::MatrixX3d>((double *)mesh->vertices_.data(),
											 mesh->vertices_.size(), 3);
			F = Eigen::Map<Eigen::MatrixX3i>((int *)mesh->triangles_.data(),
											 mesh->triangles_.size(), 3);

			std::ofstream infoFile;
			infoFile.open(f + ".info");
			infoFile << "mesh\t\t" << f << std::endl;
			infoFile << "class\t\t" << splitted[std::max((int)splitted.size() - 2, 0)] << std::endl;
			infoFile << "vertices\t" << V.rows() << std::endl;
			infoFile << "faces\t\t" << F.rows() << std::endl;
			infoFile << "facetype\t"
					 << "triangle" << std::endl; // TODO
			auto minCorner = V.colwise().minCoeff();
			auto maxCorner = V.colwise().maxCoeff();
			auto baryCenter = V.colwise().sum() / V.rows();
			auto scaleFactor = 0.5 / std::max((baryCenter - minCorner).maxCoeff(),
											  (maxCorner - baryCenter).maxCoeff());
			auto eigenvecsandvals = ComputeEigenVectors(V);
			infoFile << "mincorner\t" << minCorner << std::endl;
			infoFile << "maxcorner\t" << maxCorner << std::endl;
			infoFile << "barycenter\t" << baryCenter << std::endl;
			infoFile << "scaleFactor\t" << scaleFactor << std::endl;
			infoFile << "majoreigenvector" << eigenvecsandvals.col(0).transpose() << std::endl;
			infoFile << "mediumeigenvector" << eigenvecsandvals.col(1).transpose() << std::endl;
			infoFile << "minoreigenvector" << eigenvecsandvals.col(2).transpose() << std::endl;

			infoFile.close();

			avgV += V.rows();
			minV = std::min(minV, (int)V.rows());
			maxV = std::max(maxV, (int)V.rows());

			avgF += F.rows();
			minF = std::min(minF, (int)F.rows());
			maxF = std::max(maxF, (int)F.rows());

			if (F.rows() > 30000 || F.rows() < 10000)
				outlier.push_back(f);

			auto normalized = scaleFactor * (V.rowwise() - baryCenter);
			// TODO WRITE to file
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