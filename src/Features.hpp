#pragma once

#include <filesystem>
#include <fstream>
#include <vector>

#include "Open3D/IO/TriangleMeshIO.h"
#include "Open3D/Geometry/BoundingVolume.h"

#include "EigenVectors.hpp"
#include "Utils.hpp"

/// Basic histogram that can count items in bins and normalize this. Type T needs to support
/// operator<(T, T), operator+(T, T) and operator*(T, size_t)
template <typename T>
class Histogram
{
  private:
	std::vector<int> bins;
	T minVal;
	T binSize;
	size_t binCount;

  public:
	Histogram(T minVal, T binSize, size_t binCount)
		: minVal(minVal), binSize(binSize), binCount(binCount), bins(binCount)
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

	/// Adds items into the histogram anything less then the minVal of the histogram will be added
	/// to bin 0, anything larger then minVal + binSize * binCount will be added to the last bin.
	void AddToHistogram(const std::vector<T> &items)
	{
		std::sort(items.begin(), items.end());
		auto iter = items.begin();
		for (size_t i = 0; i < binCount; i++)
			while (*iter < minVal + binSize * (i + 1))
			{
				bins[i]++;
				if (++iter == items.end())
					return;
			}
		while (++iter != items.end())
			bins[binCount - 1]++;
	}
};

struct Features
{
	double surface_area, compactness, aabbV, diameter, eccentricity;

	int A3_size = 10, D1_size = 10, D2_size = 10, D3_size = 10, D4_size = 10;
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
	features.compactness = 0;
	features.aabbV = mesh->GetAxisAlignedBoundingBox().Volume();
	features.diameter = 0;
	Eigen::Matrix3d evecs;
	Eigen::Vector3d evals;
	std::tie(evals, evecs) = ComputeEigenValuesAndVectors(mesh->vertices_);
	features.eccentricity = evals(0) / evals(3);
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
		auto mesh = open3d::io::CreateMeshFromFile(files[i]);
		features.push_back(CalcFeatures(mesh));

		std::ofstream out(replaceDir(files[i], FEATURE_DIR).replace_extension(".features"));
		out << features[i];
		out.close();
	}
	return features;
}