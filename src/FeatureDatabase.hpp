#pragma once

#include <filesystem>
#include <fstream>
#include <vector>

#include "Open3D/IO/TriangleMeshIO.h"

#include "Features.hpp"
#include "Utils.hpp"

/// Struct containing normalized feature vectors and their corresponding meshes and statistical
/// values
struct FeatureDatabase
{
	std::vector<Features> features;
	std::vector<std::filesystem::path> meshes;
	double surface_area_avg, compactness_avg, aabbV_avg, diameter_avg, eccentricity_avg;
	double surface_area_stddev, compactness_stddev, aabbV_stddev, diameter_stddev,
		eccentricity_stddev;

	FeatureDatabase()
	{
	}

	/// Reads a feature database from the file system
	FeatureDatabase(const std::string &db_dir)
	{
		auto files = getAllFilesInDir(db_dir);
		std::sort(files.begin(), files.end(), std::greater<std::filesystem::path>());
		std::ifstream in(files[0]);
		in >> surface_area_avg >> surface_area_stddev >> compactness_stddev >> compactness_avg >>
			aabbV_avg >> aabbV_stddev >> diameter_avg >> diameter_stddev >> eccentricity_avg >>
			eccentricity_stddev >> lowerBoundF >> targetF >> upperBoundF >> avgV >> avgF >> minV >>
			maxV >> minF >> maxF >> numMeshes;
		in.close();
		features = std::vector<Features>(files.size() - 1);
		for (size_t i = 1; i < files.size(); i++)
			features[i - 1].ReadFromFile(files[i]);
		meshes = std::vector<std::filesystem::path>(files.size() - 1);
		for (size_t i = 1; i < files.size(); i++)
			meshes[i - 1] = replaceDir(files[i], ORIGINAL_DIR).replace_extension(".off");
	}

	/// Calculates a feature database and the statistics for the normalized mesh database
	FeatureDatabase(const std::vector<std::filesystem::path> &files)
		: features(files.size()), meshes(files.size())
	{
		for (size_t i = 0; i < files.size(); i++)
		{
			auto mesh = open3d::io::CreateMeshFromFile(files[i].string());
			features[i] = Features(mesh);
			meshes[i] = replaceDir(files[i], ORIGINAL_DIR);
		}

		CalculateStatisticalValues();

		// Normalize Features
		for (auto &f : features)
			NormalizeFeatures(f);

		WriteDatabaseInfo(FEATURE_DIR "/db_info.fdb");
		for (size_t i = 0; i < files.size(); i++)
		{
			std::ofstream out(replaceDir(files[i], FEATURE_DIR).replace_extension(".features"));
			out << features[i];
			out.close();
		}
	}

	/// Calculate the stddev and mean of the global features
	void CalculateStatisticalValues()
	{
		size_t n = features.size();
		// Compute Averages
		for (const auto &f : features)
		{
			surface_area_avg += f.surface_area() / n;
			compactness_avg += f.compactness() / n;
			aabbV_avg += f.aabbV() / n;
			diameter_avg += f.diameter() / n;
			eccentricity_avg += f.eccentricity() / n;
		}

		// Compute standard deviations
		// First accumulate squared mean distance
		for (const auto &f : features)
		{
			double diff = f.surface_area() - surface_area_avg;
			surface_area_stddev += diff * diff;
			diff = f.compactness() - compactness_avg;
			compactness_stddev += diff * diff;
			diff = f.aabbV() - aabbV_avg;
			aabbV_stddev += diff * diff;
			diff = f.diameter() - diameter_avg;
			diameter_stddev += diff * diff;
			diff = f.eccentricity() - eccentricity_avg;
			eccentricity_stddev += diff * diff;
		}
		// Secondly compute actual standard deviation
		surface_area_stddev = std::sqrt(surface_area_stddev / (n - 1));
		compactness_stddev = std::sqrt(compactness_stddev / (n - 1));
		aabbV_stddev = std::sqrt(aabbV_stddev / (n - 1));
		diameter_stddev = std::sqrt(diameter_stddev / (n - 1));
		eccentricity_stddev = std::sqrt(eccentricity_stddev / (n - 1));
	}

	/// Normalize a feature vector with the internally stored statistical values.
	void NormalizeFeatures(Features &f)
	{
		f.surface_area() = (f.surface_area() - surface_area_avg) / (2 * surface_area_stddev);
		f.compactness() = (f.compactness() - compactness_avg) / (2 * compactness_stddev);
		f.aabbV() = (f.aabbV() - aabbV_avg) / (2 * aabbV_stddev);
		f.diameter() = (f.diameter() - diameter_avg) / (2 * diameter_stddev);
		f.eccentricity() = (f.eccentricity() - eccentricity_avg) / (2 * eccentricity_stddev);
	}

	/// Writes a database info file containing all needed statistic about the entire mesh database.
	void WriteDatabaseInfo(const std::filesystem::path &path)
	{
		std::ofstream out(path);
		out << surface_area_avg << std::endl
			<< surface_area_stddev << std::endl
			<< compactness_stddev << std::endl
			<< compactness_avg << std::endl
			<< aabbV_avg << std::endl
			<< aabbV_stddev << std::endl
			<< diameter_avg << std::endl
			<< diameter_stddev << std::endl
			<< eccentricity_avg << std::endl
			<< eccentricity_stddev << std::endl
			<< lowerBoundF << std::endl
			<< targetF << std::endl
			<< upperBoundF << std::endl
			<< avgV << std::endl
			<< avgF << std::endl
			<< minV << std::endl
			<< maxV << std::endl
			<< minF << std::endl
			<< maxF << std::endl
			<< numMeshes << std::endl;
		out.close();
	}

	/// Calculate a distance vector for a give feature with each feature in this database, tupled
	/// with the original mesh file.
	std::vector<std::pair<double, std::filesystem::path>> CalcDistances(const Features &f)
	{
		std::vector<std::pair<double, std::filesystem::path>> distances(features.size());
		for (size_t i = 0; i < features.size(); i++)
			distances[i] = std::make_pair(features[i].Distance(f), meshes[i]);
		return distances;
	}

	/// Weight all feature vectors in this database.
	template <typename T>
	void WeightFeatures(const std::vector<T> &weights)
	{
		for (auto &f : features)
			f.WeightFeatures(weights);
	}

	/// Gets a copy of all feature vectors in a single vector weighted with provided weights.
	template <typename T>
	std::vector<double> GetWeightedFeatures(const std::vector<T> &weights)
	{
		std::vector<double> res(numMeshes * Features::size());
		for (size_t i = 0; i < numMeshes; i++)
			for (size_t j = 0; j < Features::size(); j++)
				res[i * Features::size() + j] =
					weights[Features::weightIndex(j)] * features[i].data()[j];
		return res;
	}

	/// Gets a copy of all feature vectors in a single vector.
	std::vector<double> GetFeatureVectors()
	{
		std::vector<double> res(numMeshes * Features::size());
		for (size_t i = 0; i < numMeshes; i++)
			for (size_t j = 0; j < Features::size(); j++)
				res[i * Features::size() + j] = features[i].data()[j];
		return res;
	}
};