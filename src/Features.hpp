#pragma once

#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <random>
#include <vector>

#include "emd/emd.h"

#include "Open3D/Geometry/BoundingVolume.h"
#include "Open3D/IO/TriangleMeshIO.h"

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
		: minVal(minVal), maxVal(maxVal), binSize((maxVal - minVal) / binCount), binCount(binCount),
		  bins(binCount)
	{
	}

	/// Returns the normalized histogram
	std::vector<double> Normalized()
	{
		int total = std::accumulate(bins.begin(), bins.end(), 0);
		if (!total)
			return std::vector<double>(binCount);
		std::vector<double> normalized(binCount);
		std::transform(bins.begin(), bins.end(), normalized.begin(),
					   std::bind(std::multiplies<double>(), std::placeholders::_1, 1.0 / total));
		return normalized;
	}

	/// Adds items into the histogram anything less then the minVal or greater then maxVal will be
	/// discarded
	void AddToHistogram(std::vector<T> &items)
	{
		std::sort(items.begin(), items.end());
		auto iter = items.begin();
		while (*iter < minVal)
			iter++;
		for (size_t i = 0; i < binCount; i++)
			while (*iter < minVal + binSize * (i + 1))
			{
				bins[i]++;
				if (++iter == items.end())
					return;
			}
		if (iter != items.end())
			std::cout << "[WARNING] values exceed histogram range" << std::endl;
	}
};

const std::vector<double> us_weights({2, 3, 1, 1, 2, 5, 5, 4, 1, 5});

struct Features
{
  private:
	std::vector<double> values;

  public:
	const static int A3_size = 10, D1_size = 10, D2_size = 10, D3_size = 10, D4_size = 10;

	static int size()
	{
		return 5 + A3_size + D1_size + D2_size + D3_size + D4_size;
	}

	static int weightIndex(int dataIdx)
	{
		if (dataIdx < 5)
			return dataIdx;
		else if (dataIdx < 5 + A3_size)
			return 5;
		else if (dataIdx < 5 + A3_size + D1_size)
			return 6;
		else if (dataIdx < 5 + A3_size + D1_size + D2_size)
			return 7;
		else if (dataIdx < 5 + A3_size + D1_size + D2_size + D3_size)
			return 8;
		else
			return 9;
	}

	double *data()
	{
		return values.data();
	}
	double &surface_area()
	{
		return values[0];
	}
	double surface_area() const
	{
		return values[0];
	}
	double &compactness()
	{
		return values[1];
	}
	double compactness() const
	{
		return values[1];
	}
	double &aabbV()
	{
		return values[2];
	}
	double aabbV() const
	{
		return values[2];
	}
	double &diameter()
	{
		return values[3];
	}
	double diameter() const
	{
		return values[3];
	}
	double &eccentricity()
	{
		return values[4];
	}
	double eccentricity() const
	{
		return values[4];
	}

	double *A3()
	{
		return &values[5];
	}
	const double *A3() const
	{
		return &values[5];
	}
	double *D1()
	{
		return &values[5 + A3_size];
	}
	const double *D1() const
	{
		return &values[5 + A3_size];
	}
	double *D2()
	{
		return &values[5 + A3_size + D1_size];
	}
	const double *D2() const
	{
		return &values[5 + A3_size + D1_size];
	}
	double *D3()
	{
		return &values[5 + A3_size + D1_size + D2_size];
	}
	const double *D3() const
	{
		return &values[5 + A3_size + D1_size + D2_size];
	}
	double *D4()
	{
		return &values[5 + A3_size + D1_size + D2_size + D3_size];
	}
	const double *D4() const
	{
		return &values[5 + A3_size + D1_size + D2_size + D3_size];
	}

	// A3 max pi => 3.14
	// D1 max sqrt(3)
	// D2 max sqrt(3) => 1.73 => +- 1.8
	// D3 max sqrt(sqrt(3) / 2) => 0.93 => +- 0.8
	// D4 max at most 1/2 because longest edge is sqrt(3) largest base is sqrt(3) / 2 => 1/3 *
	// sqrt(3) / 2 * sqrt(3) = 1/2 => cuberoot(1/2) = 0.79 => +- 0.6

	Features() : values(5 + A3_size + D1_size + D2_size + D3_size + D4_size)
	{
	}

	Features(const double *vals) : values(5 + A3_size + D1_size + D2_size + D3_size + D4_size)
	{
		std::memcpy(values.data(), vals, values.size());
	}

	void ReadFromFile(const std::filesystem::path &file)
	{
		std::ifstream in(file);
		in >> surface_area() >> compactness() >> aabbV() >> diameter() >> eccentricity();
		for (size_t i = 0; i < A3_size; i++)
			in >> A3()[i];
		for (size_t i = 0; i < D1_size; i++)
			in >> D1()[i];
		for (size_t i = 0; i < D2_size; i++)
			in >> D2()[i];
		for (size_t i = 0; i < D3_size; i++)
			in >> D3()[i];
		for (size_t i = 0; i < D4_size; i++)
			in >> D4()[i];
	}

	double Distance(Features other)
	{
#if 0 // EMD + L2
		const Eigen::VectorXf weights = Eigen::VectorXf::Constant(A3_size, 1);
		auto dist = [](const feature_t *left, const feature_t *right) {
			return float(std::abs(*left - *right));
		};

		signature_t A3_left = {A3_size, A3(), weights.data()};
		signature_t A3_right = {A3_size, other.A3(), weights.data()};
		double A3_dist = emd(&A3_left, &A3_right, dist, NULL, NULL);

		signature_t D1_left = {D1_size, D1(), weights.data()};
		signature_t D1_right = {D1_size, other.D1(), weights.data()};
		double D1_dist = emd(&D1_left, &D1_right, dist, NULL, NULL);

		signature_t D2_left = {D2_size, D2(), weights.data()};
		signature_t D2_right = {D2_size, other.D2(), weights.data()};
		double D2_dist = emd(&D2_left, &D2_right, dist, NULL, NULL);

		signature_t D3_left = {D3_size, D3(), weights.data()};
		signature_t D3_right = {D3_size, other.D3(), weights.data()};
		double D3_dist = emd(&D3_left, &D3_right, dist, NULL, NULL);

		signature_t D4_left = {D4_size, D4(), weights.data()};
		signature_t D4_right = {D4_size, other.D4(), weights.data()};
		double D4_dist = emd(&D4_left, &D4_right, dist, NULL, NULL);

		Eigen::VectorXd feature_1(5), feature_2(5);
		feature_1 << surface_area(), compactness(), aabbV(), diameter(), eccentricity();
		feature_2 << other.surface_area(), other.compactness(), other.aabbV(), other.diameter(),
			other.eccentricity();

		Eigen::Matrix<double, 10, 1> W(us_weights.data());
		auto glob_dist = (feature_1 - feature_2).cwiseAbs2();
		Eigen::Matrix<double, 10, 1> diff;
		diff << glob_dist, A3_dist, D1_dist, D2_dist, D3_dist, D4_dist;
		
		return diff.cwiseProduct(W).sum();

#else

		Eigen::Matrix<double, 55, 1> l(data());
		for (size_t i = 0; i < size(); i++)
			l(i) *= us_weights[weightIndex(i)];
		Eigen::Matrix<double, 55, 1> r(other.data());
		for (size_t i = 0; i < size(); i++)
			r(i) *= us_weights[weightIndex(i)];
#if 1 // L2
		return (l - r).cwiseAbs2().sum();
#else // L1
		return (l - r).cwiseAbs().sum();
#endif
#endif
	}

	template <typename T>
	void WeightFeatures(const std::vector<T> &weights)
	{
		for (size_t i = 0; i < size(); i++)
			values[i] *= weights[weightIndex(i)];
	}
};

std::ostream &operator<<(std::ostream &o, const Features &a)
{
	o << a.surface_area() << std::endl
	  << a.compactness() << std::endl
	  << a.aabbV() << std::endl
	  << a.diameter() << std::endl
	  << a.eccentricity() << std::endl;
	for (size_t i = 0; i < Features::A3_size; i++)
		o << a.A3()[i] << " ";
	o << std::endl;
	for (size_t i = 0; i < Features::D1_size; i++)
		o << a.D1()[i] << " ";
	o << std::endl;
	for (size_t i = 0; i < Features::D2_size; i++)
		o << a.D2()[i] << " ";
	o << std::endl;
	for (size_t i = 0; i < Features::D3_size; i++)
		o << a.D3()[i] << " ";
	o << std::endl;
	for (size_t i = 0; i < Features::D4_size; i++)
		o << a.D4()[i] << " ";
	o << std::endl;
	return o;
}

Features CalcFeatures(const std::shared_ptr<open3d::geometry::TriangleMesh> &mesh)
{
	Features features;

	features.surface_area() = mesh->GetSurfaceArea();

	// singed volume of a tetrahedra OABC given OA, OB and OC: V = (1/6) * dot(cross(OA, OB), OC)
	// Volume of a mesh is the sum of the signed volumes of tetrahedra contructed by its triangles
	// and the origin.
	double volume =
		std::accumulate(mesh->triangles_.begin(), mesh->triangles_.end(), 0.0,
						[=](double curr, const Eigen::Vector3i &t) {
							return curr + (1.0 / 6.0) * mesh->vertices_[t(0)]
															.cross(mesh->vertices_[t(1)])
															.dot(mesh->vertices_[t(2)]);
						});

	features.compactness() = std::cbrt(36 * M_PI * volume * volume) / features.surface_area();

	features.aabbV() = mesh->GetAxisAlignedBoundingBox().Volume();

	features.diameter() = std::sqrt(std::accumulate(
		mesh->vertices_.begin(), mesh->vertices_.end(), 0.0,
		[mesh](double mesh_max, const Eigen::Vector3d &v0) {
			return std::max(mesh_max,
							std::accumulate(mesh->vertices_.begin(), mesh->vertices_.end(), 0.0,
											[v0](double vert_max, const Eigen::Vector3d &v1) {
												return std::max(vert_max, (v1 - v0).squaredNorm());
											}));
		}));

	Eigen::Matrix3d evecs;
	Eigen::Vector3d evals;
	std::tie(evals, evecs) = ComputeEigenValuesAndVectors(mesh->vertices_);
	features.eccentricity() = evals(0) / evals(2);

	Histogram<double> A3_hist(0, 0.8 * M_PI, features.A3_size);
	std::shuffle(mesh->triangles_.begin(), mesh->triangles_.end(),
				 std::default_random_engine(std::time(0)));
	const size_t s_avgV = size_t(avgV);
	auto A3_sample = mesh->SamplePointsUniformly(s_avgV * 3);
	std::vector<double> A3_vals(s_avgV);
	for (size_t i = 0; i < s_avgV * 3; i += 3)
	{
		auto nv_ab = (A3_sample->points_[i + 1] - A3_sample->points_[i]).normalized();
		auto nv_ac = (A3_sample->points_[i + 2] - A3_sample->points_[i]).normalized();
		A3_vals[i / 3] = nv_ab.dot(nv_ac);
	}
	A3_hist.AddToHistogram(A3_vals);
	auto norm = A3_hist.Normalized();
	for (size_t i = 0; i < features.A3_size; i++)
		features.A3()[i] = norm[i];

	Histogram<double> D1_hist(0, 0.8 * 1.8, features.D1_size);
	std::shuffle(mesh->triangles_.begin(), mesh->triangles_.end(),
				 std::default_random_engine(std::time(0)));
	auto D1_sample = mesh->SamplePointsUniformly(s_avgV);
	std::vector<double> D1_vals(s_avgV);
	std::transform(D1_sample->points_.begin(), D1_sample->points_.end(), D1_vals.begin(),
				   [mesh](const Eigen::Vector3d &p) { return (mesh->GetCenter() - p).norm(); });
	D1_hist.AddToHistogram(D1_vals);
	norm = D1_hist.Normalized();
	for (size_t i = 0; i < features.D1_size; i++)
		features.D1()[i] = norm[i];

	Histogram<double> D2_hist(0, 0.8 * 1.8, features.D2_size);
	std::shuffle(mesh->triangles_.begin(), mesh->triangles_.end(),
				 std::default_random_engine(std::time(0)));
	auto D2_sample = mesh->SamplePointsUniformly(s_avgV * 2);
	std::vector<double> D2_vals(s_avgV);
	for (size_t i = 0; i < s_avgV * 2; i += 2)
		D2_vals[i / 2] = (D2_sample->points_[i] - D2_sample->points_[i + 1]).norm();
	D2_hist.AddToHistogram(D2_vals);
	norm = D2_hist.Normalized();
	for (size_t i = 0; i < features.D2_size; i++)
		features.D2()[i] = norm[i];

	Histogram<double> D3_hist(0, 0.8 * 0.8, features.D3_size);
	std::shuffle(mesh->triangles_.begin(), mesh->triangles_.end(),
				 std::default_random_engine(std::time(0)));
	auto D3_sample = mesh->SamplePointsUniformly(s_avgV * 3);
	std::vector<double> D3_vals(s_avgV);
	for (size_t i = 0; i < s_avgV * 3; i += 3)
	{
		auto v_ab = D3_sample->points_[i + 1] - D3_sample->points_[i];
		auto v_ac = D3_sample->points_[i + 2] - D3_sample->points_[i];
		D3_vals[i / 3] = std::sqrt((1.0 / 2.0) * v_ab.cross(v_ac).norm());
	}
	D3_hist.AddToHistogram(D3_vals);
	norm = D3_hist.Normalized();
	for (size_t i = 0; i < features.D3_size; i++)
		features.D3()[i] = norm[i];

	Histogram<double> D4_hist(0, 0.8 * 0.6, features.D4_size);
	std::shuffle(mesh->triangles_.begin(), mesh->triangles_.end(),
				 std::default_random_engine(std::time(0)));
	auto D4_sample = mesh->SamplePointsUniformly(s_avgV * 4);
	std::vector<double> D4_vals(s_avgV);
	//(1/6) * dot(cross(AB, AC), AD)
	for (size_t i = 0; i < s_avgV * 4; i += 4)
	{
		auto v_ab = D4_sample->points_[i + 1] - D4_sample->points_[i];
		auto v_ac = D4_sample->points_[i + 2] - D4_sample->points_[i];
		auto v_ad = D4_sample->points_[i + 3] - D4_sample->points_[i];
		D4_vals[i / 4] = std::cbrt((1.0 / 6.0) * std::abs(v_ab.cross(v_ac).dot(v_ad)));
	}
	D4_hist.AddToHistogram(D4_vals);
	norm = D4_hist.Normalized();
	for (size_t i = 0; i < features.D4_size; i++)
		features.D4()[i] = norm[i];

	return features;
}

// Struct containing normalized feature vectors
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

	FeatureDatabase(const std::vector<Features> &not_normal,
					const std::vector<std::filesystem::path> &meshes)
		: features(not_normal), meshes(meshes)
	{
		int n = not_normal.size();
		// Compute Averages
		for (const auto &f : not_normal)
		{
			surface_area_avg += f.surface_area() / n;
			compactness_avg += f.compactness() / n;
			aabbV_avg += f.aabbV() / n;
			diameter_avg += f.diameter() / n;
			eccentricity_avg += f.eccentricity() / n;
		}

		// Compute standard deviations
		// First accumulate squared mean distance
		for (const auto &f : not_normal)
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

		// Normalize Features
		for (auto &f : features)
			NormalizeFeatures(f);
	}

	void NormalizeFeatures(Features &f)
	{
		f.surface_area() = (f.surface_area() - surface_area_avg) / (2 * surface_area_stddev);
		f.compactness() = (f.compactness() - compactness_avg) / (2 * compactness_stddev);
		f.aabbV() = (f.aabbV() - aabbV_avg) / (2 * aabbV_stddev);
		f.diameter() = (f.diameter() - diameter_avg) / (2 * diameter_stddev);
		f.eccentricity() = (f.eccentricity() - eccentricity_avg) / (2 * eccentricity_stddev);
	}

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

	std::vector<std::pair<double, std::filesystem::path>> CalcDistances(const Features &f)
	{
		std::vector<std::pair<double, std::filesystem::path>> distances(features.size());
		for (size_t i = 0; i < features.size(); i++)
			distances[i] = std::make_pair(features[i].Distance(f), meshes[i]);
		return distances;
	}

	template <typename T>
	std::vector<double> GetFeatures(const std::vector<T> &weights)
	{
		std::vector<double> res(numMeshes * Features::size());
		for (size_t i = 0; i < numMeshes; i++)
			for (size_t j = 0; j < Features::size(); j++)
				res[i * Features::size() + j] =
					weights[Features::weightIndex(j)] * features[i].data()[j];
		return res;
	}
};

FeatureDatabase CalculateFeaturesMeshDatabase(const std::vector<std::filesystem::path> &files)
{
	std::vector<Features> features(files.size());
	for (size_t i = 0; i < files.size(); i++)
	{
		auto mesh = open3d::io::CreateMeshFromFile(files[i].string());
		features[i] = CalcFeatures(mesh);
	}
	std::vector<std::filesystem::path> originals(files.size());
	for (size_t i = 0; i < files.size(); i++)
		originals[i] = replaceDir(files[i], ORIGINAL_DIR);

	FeatureDatabase fdb(features, originals);

	fdb.WriteDatabaseInfo(FEATURE_DIR "/db_info.fdb");
	for (size_t i = 0; i < files.size(); i++)
	{
		std::ofstream out(replaceDir(files[i], FEATURE_DIR).replace_extension(".features"));
		out << fdb.features[i];
		out.close();
	}
	return fdb;
}