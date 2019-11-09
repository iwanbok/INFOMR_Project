#pragma once

#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <random>
#include <vector>

#include "emd/emd.h"

#include "Open3D/Geometry/BoundingVolume.h"
#include "Open3D/Geometry/TriangleMesh.h"

#include "EigenVectors.hpp"
#include "Histogram.hpp"
#include "Utils.hpp"

/// Struct representing a single mesh's feature vector
struct Features
{
  private:
	/// internal storage for feature values
	std::vector<double> values;

  public:
	/// Constant variables for bins counts of the historgrams
	const static int A3_size = 10, D1_size = 10, D2_size = 10, D3_size = 10, D4_size = 10;

	/// Total size of the feature vector
	static int size()
	{
		return 5 + A3_size + D1_size + D2_size + D3_size + D4_size;
	}

	/// Returns the index in the weights vector based on data index(in values vector)
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

	/// Pointer to feature data.
	double *data()
	{
		return values.data();
	}

	/// Reference to surface area feature
	double &surface_area()
	{
		return values[0];
	}
	double surface_area() const
	{
		return values[0];
	}

	/// Reference to compactness feature
	double &compactness()
	{
		return values[1];
	}
	double compactness() const
	{
		return values[1];
	}

	/// Reference to axis aligned bounding box volume feature
	double &aabbV()
	{
		return values[2];
	}
	double aabbV() const
	{
		return values[2];
	}

	/// Reference to diameter feature
	double &diameter()
	{
		return values[3];
	}
	double diameter() const
	{
		return values[3];
	}

	/// Reference to eccentricity feature
	double &eccentricity()
	{
		return values[4];
	}
	double eccentricity() const
	{
		return values[4];
	}

	/// Pointer to start of A3 histogram
	double *A3()
	{
		return &values[5];
	}
	const double *A3() const
	{
		return &values[5];
	}

	/// Pointer to start of D1 histogram
	double *D1()
	{
		return &values[5 + A3_size];
	}
	const double *D1() const
	{
		return &values[5 + A3_size];
	}

	/// Pointer to start of D2 histogram
	double *D2()
	{
		return &values[5 + A3_size + D1_size];
	}
	const double *D2() const
	{
		return &values[5 + A3_size + D1_size];
	}

	/// Pointer to start of D3 histogram
	double *D3()
	{
		return &values[5 + A3_size + D1_size + D2_size];
	}
	const double *D3() const
	{
		return &values[5 + A3_size + D1_size + D2_size];
	}

	/// Pointer to start of D4 histogram
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

	/// Calculates the feature vector for the provided mesh
	Features(const std::shared_ptr<open3d::geometry::TriangleMesh> &mesh)
		: values(5 + A3_size + D1_size + D2_size + D3_size + D4_size)
	{
		surface_area() = mesh->GetSurfaceArea();

		// singed volume of a tetrahedra OABC given OA, OB and OC: V = (1/6) * dot(cross(OA, OB),
		// OC) Volume of a mesh is the sum of the signed volumes of tetrahedra contructed by its
		// triangles and the origin.
		double volume =
			std::accumulate(mesh->triangles_.begin(), mesh->triangles_.end(), 0.0,
							[=](double curr, const Eigen::Vector3i &t) {
								return curr + (1.0 / 6.0) * mesh->vertices_[t(0)]
																.cross(mesh->vertices_[t(1)])
																.dot(mesh->vertices_[t(2)]);
							});

		compactness() = std::cbrt(36 * M_PI * volume * volume) / surface_area();

		aabbV() = mesh->GetAxisAlignedBoundingBox().Volume();

		diameter() = std::sqrt(std::accumulate(
			mesh->vertices_.begin(), mesh->vertices_.end(), 0.0,
			[mesh](double mesh_max, const Eigen::Vector3d &v0) {
				return std::max(mesh_max,
								std::accumulate(mesh->vertices_.begin(), mesh->vertices_.end(), 0.0,
												[v0](double vert_max, const Eigen::Vector3d &v1) {
													return std::max(vert_max,
																	(v1 - v0).squaredNorm());
												}));
			}));

		Eigen::Matrix3d evecs;
		Eigen::Vector3d evals;
		std::tie(evals, evecs) = ComputeEigenValuesAndVectors(mesh->vertices_);
		eccentricity() = evals(0) / evals(2);

		auto random_engine = std::default_random_engine((unsigned int)std::time(0));

		Histogram<double> A3_hist(0, 0.8 * M_PI, A3_size);
		std::shuffle(mesh->triangles_.begin(), mesh->triangles_.end(), random_engine);
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
		for (size_t i = 0; i < A3_size; i++)
			A3()[i] = norm[i];

		Histogram<double> D1_hist(0, 0.8 * 1.8, D1_size);
		std::shuffle(mesh->triangles_.begin(), mesh->triangles_.end(), random_engine);
		auto D1_sample = mesh->SamplePointsUniformly(s_avgV);
		std::vector<double> D1_vals(s_avgV);
		std::transform(D1_sample->points_.begin(), D1_sample->points_.end(), D1_vals.begin(),
					   [mesh](const Eigen::Vector3d &p) { return (mesh->GetCenter() - p).norm(); });
		D1_hist.AddToHistogram(D1_vals);
		norm = D1_hist.Normalized();
		for (size_t i = 0; i < D1_size; i++)
			D1()[i] = norm[i];

		Histogram<double> D2_hist(0, 0.8 * 1.8, D2_size);
		std::shuffle(mesh->triangles_.begin(), mesh->triangles_.end(), random_engine);
		auto D2_sample = mesh->SamplePointsUniformly(s_avgV * 2);
		std::vector<double> D2_vals(s_avgV);
		for (size_t i = 0; i < s_avgV * 2; i += 2)
			D2_vals[i / 2] = (D2_sample->points_[i] - D2_sample->points_[i + 1]).norm();
		D2_hist.AddToHistogram(D2_vals);
		norm = D2_hist.Normalized();
		for (size_t i = 0; i < D2_size; i++)
			D2()[i] = norm[i];

		Histogram<double> D3_hist(0, 0.8 * 0.8, D3_size);
		std::shuffle(mesh->triangles_.begin(), mesh->triangles_.end(), random_engine);
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
		for (size_t i = 0; i < D3_size; i++)
			D3()[i] = norm[i];

		Histogram<double> D4_hist(0, 0.8 * 0.6, D4_size);
		std::shuffle(mesh->triangles_.begin(), mesh->triangles_.end(), random_engine);
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
		for (size_t i = 0; i < D4_size; i++)
			D4()[i] = norm[i];
	}

	/// Reads a feature vector from a given file
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

	/// Calculates the distance between this feature vector and another via a compile time constant
	/// set method.
	double Distance(Features other)
	{
#ifdef EMD // EMD + L2
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

		auto glob_dist = (feature_1 - feature_2).cwiseAbs2();
		
		return std::sqrt(glob_dist.sum()) + A3_dist + D1_dist + D2_dist + D3_dist + D4_dist;

#else
		Eigen::Matrix<double, 55, 1> l(data());
		Eigen::Matrix<double, 55, 1> r(other.data());
 #ifdef L2  // L2
		return std::sqrt((l - r).cwiseAbs2().sum());
 #else      // L1
		return (l - r).cwiseAbs().sum();
 #endif L2
#endif EMD
	}

	/// Weight this feature vector with provided weights
	template <typename T>
	void WeightFeatures(const std::vector<T> &weights)
	{
		for (int i = 0; i < size(); i++)
			values[i] *= weights[weightIndex(i)];
	}
};

/// Ostream push operator for Features class
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