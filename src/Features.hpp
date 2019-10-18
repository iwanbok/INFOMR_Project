#pragma once

#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <vector>

#include "Open3D/IO/TriangleMeshIO.h"
#include "Open3D/Geometry/BoundingVolume.h"

#include "EigenVectors.hpp"
#include "Utils.hpp"

/// Basic histogram that can count items in bins and normalize this. Type T needs to support
/// operator<(T, T), operator+(T, T), operator*(T, size_t) and operator/(T, size_t)
template <typename T>
class Histogram
{
  private:
	std::vector<int> bins;
	T minVal, maxVal, binSize;
	size_t binCount;

  public:
	Histogram(T minVal, T maxVal, size_t binCount)
		: minVal(minVal), maxVal(maxVal), binSize((maxVal - minVal) / binCount), binCount(binCount), bins(binCount)
	{
	}

	/// Returns the normalized histogram
	std::vector<float> Normalized()
	{
		int total = std::accumulate(bins.begin(), bins.end(), 0);
		if (!total)
			return std::vector<float>(binCount);
		std::vector<float> normalized(binCount);
		std::transform(bins.begin(), bins.end(), normalized.begin(),
					   std::bind(std::multiplies<float>(), std::placeholders::_1, 1.f / total));
		return normalized;
	}

	/// Adds items into the histogram anything less then the minVal or greater then maxVal will be discarded
	void AddToHistogram(const std::vector<T> &items)
	{
		std::sort(items.begin(), items.end());
		auto iter = items.begin();
		while(*iter < minVal)
			iter++;
		for (size_t i = 0; i < binCount; i++)
			while (*iter < minVal + binSize * (i + 1))
			{
				bins[i]++;
				if (++iter == items.end())
					return;
			}
		if(iter != items.end())
			std::cout << "[WARNING] values exceed histogram range" << std::endl;
	}
};

struct Features
{
	double surface_area, compactness, aabbV, diameter, eccentricity;

	int A3_size = 10, D1_size = 10, D2_size = 10, D3_size = 10, D4_size = 10;

	// A3 max pi => 3.14
	// D1 max sqrt(3)
	// D2 max sqrt(3) => 1.73 => +- 1.8
	// D3 max sqrt(sqrt(3) / 2) => 0.93 => +- 1
	// D4 max at most 1/2 because longest edge is sqrt(3) largest base is sqrt(3) / 2 => 1/3 * sqrt(3) / 2 * sqrt(3) = 1/2 => cuberoot(1/2) = 0.79 => +- 0.8
	std::vector<float> A3, D1, D2, D3, D4;

	Features() : A3(A3_size), D1(D1_size), D2(D2_size), D3(D3_size), D4(D4_size)
	{
	}

	void ReadFromFile(const std::filesystem::path &file)
	{
		std::ifstream in(file);
		in >> surface_area >> compactness >> aabbV >> diameter >> eccentricity;
		for (size_t i = 0; i < A3_size; i++)
			in >> A3[i];
		for (size_t i = 0; i < D1_size; i++)
			in >> D1[i];
		for (size_t i = 0; i < D2_size; i++)
			in >> D2[i];
		for (size_t i = 0; i < D3_size; i++)
			in >> D3[i];
		for (size_t i = 0; i < D4_size; i++)
			in >> D4[i];
	}
};

std::ostream &operator<<(std::ostream &o, const Features &a)
{
	o << a.surface_area << std::endl
		<< a.compactness << std::endl
		<< a.aabbV << std::endl
		<< a.diameter << std::endl
		<< a.eccentricity << std::endl;
	for (size_t i = 0; i < a.A3_size; i++)
		o << a.A3[i] << " ";
	o << std::endl;
	for (size_t i = 0; i < a.D1_size; i++)
		o << a.D1[i] << " ";
	o << std::endl;
	for (size_t i = 0; i < a.D2_size; i++)
		o << a.D2[i] << " ";
	o << std::endl;
	for (size_t i = 0; i < a.D3_size; i++)
		o << a.D3[i] << " ";
	o << std::endl;
	for (size_t i = 0; i < a.D4_size; i++)
		o << a.D4[i] << " ";
	o << std::endl;
	return o;
}

Features CalcFeatures(const std::shared_ptr<open3d::geometry::TriangleMesh> &mesh)
{
	Features features;
	features.surface_area = mesh->GetSurfaceArea();
	// singed volume of a tetrahedra OABC given OA, OB and OC: V = (1/6) * dot(cross(OA, OB), OC)
	// Volume of a mesh is the sum of the signed volumes of tetrahedra contructed by its triangles and the origin.
	double volume = std::accumulate(mesh->triangles_.begin(), mesh->triangles_.end(), 0.0, [=](double curr, const Eigen::Vector3i &t)
	{
		return curr + (1.0/6.0) * mesh->vertices_[t(0)].cross(mesh->vertices_[t(1)]).dot(mesh->vertices_[t(2)]);
	});
	features.compactness = std::cbrt(36 * M_PI * volume * volume) / features.surface_area;
	features.aabbV = mesh->GetAxisAlignedBoundingBox().Volume();
	features.diameter = std::sqrt(std::accumulate(mesh->vertices_.begin(), mesh->vertices_.end(), 0.0, [=](double mesh_max, const Eigen::Vector3d &v0) 
	{
		return std::max(mesh_max, std::accumulate(mesh->vertices_.begin(), mesh->vertices_.end(), 0.0, [=](double vert_max, const Eigen::Vector3d& v1) 
		{ 
			return std::max(vert_max, (v1 - v0).squaredNorm()); 
		}));
	}));
	Eigen::Matrix3d evecs;
	Eigen::Vector3d evals;
	std::tie(evals, evecs) = ComputeEigenValuesAndVectors(mesh->vertices_);
	features.eccentricity = evals(0) / evals(2);
	Histogram<float> A3(0, 1, features.A3_size);
	features.A3 = A3.Normalized();
	Histogram<float> D1(0, 1, features.D1_size);
	features.D1 = D1.Normalized();
	Histogram<float> D2(0, 1, features.D2_size);
	features.D2 = D2.Normalized();
	Histogram<float> D3(0, 1, features.D3_size);
	features.D3 = D3.Normalized();
	Histogram<float> D4(0, 1, features.D4_size);
	features.D4 = D4.Normalized();
	return features;
}

std::vector<Features> ReadFeatureDatabase(const std::vector<std::filesystem::path> &files)
{
	std::vector<Features> features(files.size());
	for (size_t i = 0; i < files.size(); i++)
		features[i].ReadFromFile(files[i]);

	return features;
}

std::vector<Features> CalculateFeaturesMeshDatabase(const std::vector<std::filesystem::path> &files)
{
	std::vector<Features> features(files.size());
	for (size_t i = 0; i < files.size(); i++)
	{
		auto mesh = open3d::io::CreateMeshFromFile(files[i].string());
		features[i] = CalcFeatures(mesh);

		std::ofstream out(replaceDir(files[i], FEATURE_DIR).replace_extension(".features"));
		out << features[i];
		out.close();
	}
	return features;
}