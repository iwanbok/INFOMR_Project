#ifndef ASSET_PATH
#define ASSET_PATH "data/"
#endif

#define ORIGINAL_DIR ASSET_PATH "LabeledDB_new"
#define PREPROCESSED_DIR ASSET_PATH "preprocessed"
#define NORMALIZED_DIR ASSET_PATH "normalized"
#define FEATURE_DIR ASSET_PATH "features"
#define INFO_DIR ASSET_PATH "info"

#include <igl/opengl/MeshGL.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/read_triangle_mesh.h>
#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
#include <pfd/portable_file_dialogs.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <ANN/ANN.h>

#include "Open3D/Geometry/TriangleMesh.h"
#include "Open3D/IO/TriangleMeshIO.h"

#include "tsne/tsne.h"

#include "Features.hpp"
#include "Normalize.hpp"
#include "PreProcess.hpp"

using namespace Eigen;
using namespace std;

FeatureDatabase fdb;
MatrixXd V;
MatrixXi F;
vector<string> classes;
map<string, int> ccs;

const int scrWidth = 1280, scrHeight = 800;
const int xStart = 280, yStart = 0;
const int pageSize = 9;
int page = 0;
vector<vector<pair<double, filesystem::path>>> pages;

double r_ann = 1.0;
int k_ann = 20;
double r_us = 0.45;
int k_us = 20;
std::vector<int> ann_weights({1, 1, 1, 1, 1, 1, 1, 1, 1, 1});

MatrixXd distances;

class CustomMenu : public igl::opengl::glfw::imgui::ImGuiMenu
{

  public:
	ANNkd_tree f_tree;
	int TP = 0, FP = 0;
	const char *curr_class = NULL;

	CustomMenu(double *pa, int n) : f_tree(pa, n, Features::size())
	{
	}

	void reset_camera(igl::opengl::ViewerCore &c)
	{
		c.camera_zoom = 1;
		c.trackball_angle = Eigen::Quaternionf::Identity();

		// Camera parameters
		c.camera_base_zoom = 1.0f;
		c.camera_zoom = 1.0f;
		c.orthographic = false;
		c.camera_view_angle = 45.0;
		c.camera_dnear = 1.0;
		c.camera_dfar = 100.0;
		c.camera_base_translation << 0, 0, 0;
		c.camera_translation << 0, 0, 0;
		c.camera_eye << 0, 0, 5;
		c.camera_center << 0, 0, 0;
		c.camera_up << 0, 1, 0;
	}

	void set_result_meshes()
	{
		for (size_t i = 0; i < pageSize; i++)
		{
			viewer->data_list[i + 1].clear();
			if (i >= pages[page].size())
				continue;
			viewer->data_list[i + 1].set_visible(true, viewer->core_list[i + 1].id);
			int dist;
			filesystem::path mesh;
			tie(dist, mesh) = pages[page][i];
			igl::read_triangle_mesh(mesh.string(), V, F);
			viewer->data_list[i + 1].set_mesh(V, F);

			// Resetting camera parameters
			reset_camera(viewer->core_list[i + 1]);
		}
		viewer->core_list[0].viewport = Vector4f(0, 0, xStart, scrHeight);
		viewer->data_list[0].set_visible(false, viewer->core_list[0].id);
	}

	void setup_pages_and_stats(const pair<double, filesystem::path> &d)
	{
		if (pages[pages.size() - 1].size() >= pageSize)
			pages.push_back(vector<pair<double, filesystem::path>>());
		pages[pages.size() - 1].push_back(d);
		if (curr_class)
		{
			auto p = d.second.parent_path();
			auto m_class = p.string().substr(p.parent_path().string().size() + 1);
			if (strcmp(curr_class, m_class.c_str()))
				FP++;
			else
				TP++;
		}
		cout << d.first << ": " << d.second << endl;
	}

	virtual void draw_viewer_menu() override
	{
		// Draw parent menu
		// ImGuiMenu::draw_viewer_menu();

		// Viewing options
		if (ImGui::CollapsingHeader("Viewing Options", ImGuiTreeNodeFlags_DefaultOpen))
		{
			/*if (ImGui::Button("Fit model to viewport", ImVec2(-1, 0)))
			{
				int data_id = viewer->core_index(viewer->core().id);
				viewer->core().align_camera_center(viewer->data_list[data_id].V,
												   viewer->data_list[data_id].F);
			}*/
			if (ImGui::Button("Snap canonical view", ImVec2(-1, 0)))
				viewer->snap_to_canonical_quaternion();
			if (ImGui::Button("Reset camera", ImVec2(-1, 0)))
				reset_camera(viewer->core());

				// Zoom
				ImGui::PushItemWidth(80 * menu_scaling());
			ImGui::DragFloat("Zoom", &(viewer->core().camera_zoom), 0.05f, 0.1f, 20.0f);

			// Select rotation type
			int rotation_type = static_cast<int>(viewer->core().rotation_type);
			static Eigen::Quaternionf trackball_angle = Eigen::Quaternionf::Identity();
			static bool orthographic = true;
			if (ImGui::Combo("Camera Type", &rotation_type, "Trackball\0Two Axes\0002D Mode\0\0"))
			{
				using RT = igl::opengl::ViewerCore::RotationType;
				auto new_type = static_cast<RT>(rotation_type);
				for (auto &c : viewer->core_list)
					if (new_type != c.rotation_type)
					{
						if (new_type == RT::ROTATION_TYPE_NO_ROTATION)
						{
							trackball_angle = c.trackball_angle;
							orthographic = c.orthographic;
							c.trackball_angle = Eigen::Quaternionf::Identity();
							c.orthographic = true;
						}
						else if (c.rotation_type == RT::ROTATION_TYPE_NO_ROTATION)
						{
							c.trackball_angle = trackball_angle;
							c.orthographic = orthographic;
						}
						c.set_rotation_type(new_type);
					}
			}

			// Orthographic view
			ImGui::Checkbox("Orthographic view", &(viewer->core().orthographic));
			ImGui::PopItemWidth();
		}

		// Draw options
		if (ImGui::CollapsingHeader("Draw Options", ImGuiTreeNodeFlags_DefaultOpen))
		{
			static bool face_based = false;
			if (ImGui::Checkbox("Face-based", &face_based))
			{
				for (auto &d : viewer->data_list)
				{
					d.face_based = face_based;
					d.dirty = igl::opengl::MeshGL::DIRTY_ALL;
				}
			}
			ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.3f);
			static float shininess = 50.f;
			if (ImGui::DragFloat("Shininess", &shininess, 0.05f, 0.0f, 100.0f))
				for (auto &d : viewer->data_list)
					d.shininess = shininess;
			ImGui::PopItemWidth();

			static bool wireframe = true;
			if (ImGui::Checkbox("Wireframe", &wireframe))
				for (size_t i = 0; i <= pageSize; i++)
					viewer->core_list[i].set(viewer->data_list[i].show_lines, wireframe);

			static bool fill = true;
			if (ImGui::Checkbox("Fill", &fill))
				for (size_t i = 0; i <= pageSize; i++)
					viewer->core_list[i].set(viewer->data_list[i].show_faces, fill);
		}

		// Add new group
		if (ImGui::CollapsingHeader("Search Options", ImGuiTreeNodeFlags_DefaultOpen))
		{
			static shared_ptr<pfd::open_file> open_file;
			static string filename = "";

			ImGui::PushItemWidth(ImGui::GetWindowWidth() / 1.75f);
			if (ImGui::BeginCombo("Class", curr_class)) // The second parameter is the label
														// previewed before opening the combo.
			{
				for (int n = 0; n < classes.size(); n++)
				{
					bool is_selected =
						(curr_class ==
						 classes[n].c_str()); // You can store your selection however you
											  // want, outside or inside your objects
					if (ImGui::Selectable(classes[n].c_str(), is_selected))
						curr_class = classes[n].c_str();
					if (is_selected)
						ImGui::SetItemDefaultFocus(); // You may set the initial focus when opening
													  // the combo (scrolling + for keyboard
													  // navigation support)
				}
				ImGui::EndCombo();
			}

			const char *algs[] = {"K-nearest ANN", "R-nearest ANN", "K-nearest Custom",
								  "R-nearest Custom"};
			static const char *curr_alg = algs[0];
			if (ImGui::BeginCombo("Algorithm", curr_alg))
			{
				for (int n = 0; n < IM_ARRAYSIZE(algs); n++)
				{
					bool is_selected = (curr_alg == algs[n]);
					if (ImGui::Selectable(algs[n], is_selected))
						curr_alg = algs[n];
					if (is_selected)
						ImGui::SetItemDefaultFocus();
				}
				ImGui::EndCombo();
			}
			ImGui::PopItemWidth();

			const double minDrag = 0.1, maxDrag = 5.0;
			if (curr_alg == algs[0])
				ImGui::DragInt("Number of results", &k_ann, 1.0f, 1, 100);
			else if (curr_alg == algs[1])
				ImGui::DragScalar("Radius threshold", ImGuiDataType_Double, &r_ann, 0.05f, &minDrag,
								  &maxDrag, "%.3f");
			else if (curr_alg == algs[2])
				ImGui::DragInt("Number of results", &k_us, 1.0f, 1, 100);
			else if (curr_alg == algs[3])
				ImGui::DragScalar("Radius threshold", ImGuiDataType_Double, &r_us, 0.05f, &minDrag,
								  &maxDrag, "%.3f");

			ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
			if (ImGui::Button("Open File"))
				open_file = std::make_shared<pfd::open_file>(
					"Choose mesh", ORIGINAL_DIR,
					vector<string>({"3D Meshes (.off, .ply, .stl)", "*.off | *.ply | *.stl"}));
			if (open_file && open_file->ready())
			{
				auto result = open_file->result();
				if (result.size())
				{
					filename = result[0];

					igl::read_triangle_mesh(filename, V, F);
					page = 0;
					pages.clear();
					viewer->data_list[0].clear();
					viewer->data_list[0].set_mesh(V, F);
					viewer->core_list[0].viewport = Vector4f(0, 0, scrWidth, scrHeight);
					viewer->data_list[0].set_visible(true, viewer->core_list[0].id);
					for (size_t i = 0; i < pageSize; i++)
						viewer->data_list[i + 1].set_visible(false, viewer->core_list[i + 1].id);
				}
				open_file = nullptr;
			}
			ImGui::PopItemFlag();

			if (filename.size())
			{
				ImGui::SameLine();
				ImGui::Text(filesystem::path(filename).filename().string().c_str());
				if (ImGui::Button("Search"))
				{
					std::cout << "Opened file " << filename << "\n";
					auto mesh = open3d::io::CreateMeshFromFile(filename);
					PreProcess(mesh);
					Normalize(mesh);
					auto features = CalcFeatures(mesh);
					fdb.NormalizeFeatures(features);

					pages.push_back(vector<pair<double, filesystem::path>>());
					TP = 0, FP = 0;
					if (curr_alg == algs[0])
					{
						vector<int> nn_idx(k_ann);
						vector<double> dd(k_ann);
						features.WeightFeatures(ann_weights);
						f_tree.annkSearch(features.data(), k_ann, nn_idx.data(), dd.data());
						for (size_t i = 0; i < k_ann; i++)
							setup_pages_and_stats(make_pair(dd[i], fdb.meshes[nn_idx[i]]));
					}
					else if (curr_alg == algs[1])
					{
						vector<int> nn_idx(numMeshes);
						vector<double> dd(numMeshes);
						features.WeightFeatures(ann_weights);
						f_tree.annkSearch(features.data(), numMeshes, nn_idx.data(), dd.data());
						for (size_t i = 0; i < numMeshes; i++)
						{
							if (dd[i] > r_ann)
								break;
							setup_pages_and_stats(make_pair(dd[i], fdb.meshes[nn_idx[i]]));
						}
					}
					else if (curr_alg == algs[2])
					{
						auto dists = fdb.CalcDistances(features);
						sort(dists.begin(), dists.end());
						for (size_t i = 0; i < k_us; i++)
							setup_pages_and_stats(dists[i]);
					}
					else if (curr_alg == algs[3])
					{
						auto dists = fdb.CalcDistances(features);
						sort(dists.begin(), dists.end());
						for (const auto &d : dists)
						{
							if (d.first > r_us)
								break;
							setup_pages_and_stats(d);
						}
					}

					if (curr_class)
					{
						int P = ccs[string(curr_class)];
						int N = numMeshes - P;
						int TN = N - FP;
						int FN = P - TP;

						cout << "Class: " << curr_class << endl
							 << "Class size: " << P << endl
							 << "TP: " << TP << endl
							 << "FP: " << FP << endl
							 << "TN: " << TN << endl
							 << "FN: " << FN << endl
							 << "Recall: " << double(TP) / P << endl
							 << "Selectivity: " << double(TN) / N << endl
							 << "Precision: " << TP / double(TP + FP) << endl
							 << "NPV: " << TN / double(TN + FN) << endl
							 << "Threat Score: " << TP / double(TP + FN + FP) << endl
							 << "Accuracy: " << double(TP + TN) / numMeshes << endl
							 << "F1 Score: " << double(2 * TP) / double(2 * TP + FP + FN) << endl;
					}

					set_result_meshes();
					filename = "";
					curr_class = NULL;
				}
			}

			if (pages.size())
			{
				if (page > 0)
				{
					if (ImGui::ArrowButton("decPage", ImGuiDir_Left))
					{
						page = (page - 1) % pages.size();
						set_result_meshes();
					}
					ImGui::SameLine();
				}
				ImGui::Text("Page %i", page + 1);
				if (page < pages.size() - 1)
				{
					ImGui::SameLine();
					if (ImGui::ArrowButton("incPage", ImGuiDir_Right))
					{
						page = (page + 1) % pages.size();
						set_result_meshes();
					}
				}
			}
		}
	}
};

bool key_down(igl::opengl::glfw::Viewer &view, unsigned char key, int modifier)
{
	return false;
}

int main(int argc, char *argv[])
{
	string options = GetCmdOptions(argc, argv);
	if (options.find('h') != options.npos)
	{
		cout << "Command line options:" << endl
			 << "\t-p: Skip database preprocessing" << endl
			 << "\t-n: Skip database normalization" << endl
			 << "\t-f: Skip database feature calculations" << endl;
		return EXIT_SUCCESS;
	}
	if (options.find('p') == options.npos)
	{
		vector<filesystem::path> originals = getAllFilesInDir(ORIGINAL_DIR);
		if (!filesystem::exists(PREPROCESSED_DIR))
			filesystem::create_directories(PREPROCESSED_DIR);
		PreProcessMeshDatabase(originals);
	}

	if (options.find('n') == options.npos)
	{
		vector<filesystem::path> preprossed = getAllFilesInDir(PREPROCESSED_DIR);
		if (!filesystem::exists(NORMALIZED_DIR))
			filesystem::create_directories(NORMALIZED_DIR);
		NormalizeMeshDataBase(preprossed);
	}

	if (options.find('f') == options.npos)
	{
		if (options.find('n') != options.npos)
			CalcMeshStatistics(getAllFilesInDir(NORMALIZED_DIR));
		vector<filesystem::path> normalized = getAllFilesInDir(NORMALIZED_DIR);
		sort(normalized.begin(), normalized.end(), greater<filesystem::path>());
		if (!filesystem::exists(FEATURE_DIR))
			filesystem::create_directories(FEATURE_DIR);
		fdb = CalculateFeaturesMeshDatabase(normalized);
	}
	else
		fdb = FeatureDatabase(FEATURE_DIR);

	distances = MatrixXd::Zero(numMeshes, numMeshes);
	if (options.find('d') == options.npos)
	{
		ofstream out(ASSET_PATH "/distances.txt");
		for (size_t i = 0; i < numMeshes; i++)
		{
			auto dists = fdb.CalcDistances(fdb.features[i]);
			for (size_t j = 0; j < numMeshes; j++)
			{
				double dist;
				filesystem::path m;
				tie(dist, m) = dists[j];
				out << dist << " ";
				distances(i, j) = dist;
			}
			out << endl;
		}
		out.close();
	}
	else
	{
		ifstream dist_file(ASSET_PATH "/distances.txt");
		for (size_t i = 0; i < numMeshes; i++)
			for (size_t j = 0; j < numMeshes; j++)
				dist_file >> distances(i, j);
		dist_file.close();
	}

	std::vector<std::filesystem::path> files = getAllFilesInDir(PREPROCESSED_DIR);
	for (auto &f : files)
	{
		auto parent = f.parent_path();
		auto mesh_class = parent.string().substr(parent.parent_path().string().size() + 1);
		ccs[mesh_class]++;
	}
	for (map<string, int>::iterator it = ccs.begin(); it != ccs.end(); ++it)
		classes.push_back(it->first);

	/*double **buf2d;
	const int dim = 5;
	const int n = 20;
	buf2d = new double *[n];
	for (size_t i = 0; i < n; i++)
		buf2d[i] = new double[dim];

	for (size_t i = 0; i < n; i++)
		for (size_t j = 0; j < dim; j++)
			buf2d[i][j] = fdb.features[200 + i].data()[j];

	ANNkd_tree test(buf2d, n, dim);
	double *searchVal = new double[dim];
	for (size_t j = 0; j < dim; j++)
		searchVal[j] = fdb.features[210].data()[j];
	int k = 5;
	vector<int> nn_idx(k);
	vector<double> dd(k);
	test.annkSearch(searchVal, k, nn_idx.data(), dd.data());

	for (size_t i = 0; i < k; i++)
		cout << dd[i] << ": " << nn_idx[i] << endl;

	for (size_t i = 0; i < n; i++)
		delete[] buf2d[i];
	delete[] buf2d;*/

	if (options.find('w') == options.npos)
	{
		double best_prec = 0;
		const int k = 20;
		int i1 = 2, i2 = 3, i3 = 1, i4 = 1, i5 = 2, i9 = 1;
		for (int i6 = 1; i6 < 10; i6++)
			for (int i7 = 1; i7 < 10; i7++)
				for (int i8 = 1; i8 < 10; i8++)
					for (int i10 = 1; i10 < 10; i10++)
					{
						vector<int> curr_weights({i1, i2, i3, i4, i5, i6, i7, i8, i9, i10});
						auto w_feat = fdb.GetFeatures(curr_weights);
						CustomMenu menu(w_feat.data(), numMeshes);
						double precision = 0;
						for (size_t i = 0; i < numMeshes; i++)
						{
							auto parent = fdb.meshes[i].parent_path();
							auto curr_class =
								parent.string().substr(parent.parent_path().string().size() + 1);
							int FP = 0, TP = 0;
							vector<int> nn_idx(k);
							vector<double> dd(k);
							menu.f_tree.annkSearch(&w_feat[i * Features::size()], k, nn_idx.data(),
												   dd.data());
							for (int id : nn_idx)
							{
								auto p = fdb.meshes[id].parent_path();
								auto m_class =
									p.string().substr(p.parent_path().string().size() + 1);
								if (curr_class.compare(m_class))
									FP++;
								else
									TP++;
							}
							precision += TP / double(TP + FP);
						}
						precision /= numMeshes;
						if (precision > best_prec)
						{
							best_prec = precision;
							ann_weights = curr_weights;
						}
					}
		ofstream weight_file(ASSET_PATH "/weights.txt");
		for (size_t i = 0; i < 10; i++)
			weight_file << ann_weights[i] << endl;
		weight_file.close();
	}
	else
	{
		ifstream weight_file(ASSET_PATH "/weights.txt");
		for (size_t i = 0; i < 10; i++)
			weight_file >> ann_weights[i];
		weight_file.close();
	}

	auto feats = fdb.GetFeatures(ann_weights);
	CustomMenu menu(feats.data(), fdb.features.size());
	map<string, pair<double, double>> classPerf;

	for (size_t i = 0; i < numMeshes; i++)
	{
		auto parent = fdb.meshes[i].parent_path();
		auto curr_class = parent.string().substr(parent.parent_path().string().size() + 1);
		int FP = 0, TP = 0;
#if 1 // ANN
		vector<int> nn_idx(k_ann);
		vector<double> dd(k_ann);
		menu.f_tree.annkSearch(&feats[i * Features::size()], k_ann, nn_idx.data(), dd.data());
		for (int id : nn_idx)
		{
			auto p = fdb.meshes[id].parent_path();
			auto m_class = p.string().substr(p.parent_path().string().size() + 1);
			if (curr_class.compare(m_class))
				FP++;
			else
				TP++;
		}
#else
		auto dists = fdb.CalcDistances(fdb.features[i]);
		sort(dists.begin(), dists.end());
		for (size_t i = 0; i < k_us; i++)
		{
			auto p = dists[i].second.parent_path();
			auto m_class = p.string().substr(p.parent_path().string().size() + 1);
			if (curr_class.compare(m_class))
				FP++;
			else
				TP++;
		}
#endif
		int P = ccs[curr_class];
		classPerf[curr_class].first += double(TP) / P;
		classPerf[curr_class].second += TP / double(TP + FP);
	}

	double recall = 0, precision = 0;
	ofstream perf_file(ASSET_PATH "/perf.txt");
	for (const auto &cs : ccs)
	{
		recall += classPerf[cs.first].first;
		precision += classPerf[cs.first].second;
		classPerf[cs.first].first /= cs.second;
		classPerf[cs.first].second /= cs.second;
		perf_file << cs.first << " " << classPerf[cs.first].first << " "
				  << classPerf[cs.first].second << endl;
		cout << cs.first << " " << classPerf[cs.first].first << " " << classPerf[cs.first].second
			 << endl;
	}
	recall /= fdb.features.size();
	precision /= fdb.features.size();
	perf_file << "total " << recall << " " << precision << endl;
	perf_file.close();
	cout << "Recall over entire database: " << recall << endl
		 << "Precision over entire database: " << precision << endl;

	igl::opengl::glfw::Viewer viewer;
	for (size_t i = 0; i <= pageSize; i++)
		viewer.append_mesh();
	viewer.callback_init = [&](igl::opengl::glfw::Viewer &) {
		viewer.core().viewport = Vector4f(0, 0, scrWidth, scrHeight);
		const int xCount = int(ceil(pageSize / 3.0));
		const int yCount = 3;
		assert(xCount * yCount <= 31);
		const int cellWidth = (scrWidth - xStart) / xCount;
		const int cellHeight = (scrHeight - yStart) / yCount;
		viewer.core().viewport = Vector4f(0, 0, scrWidth, scrHeight);
		for (size_t j = 1; j <= 3; j++)
			for (size_t i = 0; i < ceil(pageSize / 3.0); i++)
				viewer.append_core(Vector4f(xStart + cellWidth * i,
											scrHeight - (yStart + cellHeight * j), cellWidth,
											cellHeight));

		for (auto &data : viewer.data_list)
			for (size_t i = 0; i <= pageSize; i++)
				data.set_visible(false, viewer.core_list[i].id);

		for (size_t i = 0; i <= pageSize; i++)
		{
			viewer.data_list[i].set_visible(true, viewer.core_list[i].id);
		}
		return false;
	};

	viewer.callback_key_down = &key_down;
	viewer.plugins.push_back(&menu);
	viewer.launch();

	if (options.find('s') == options.npos)
	{
		auto why_you_edit_pointer = fdb.GetFeatures(ann_weights);
		int out_dim = 2;
		double *out_vec = (double *)malloc(numMeshes * out_dim * sizeof(double));
		double *costs = (double *)calloc(numMeshes, sizeof(double));
		TSNE::run(why_you_edit_pointer.data(), numMeshes, Features::size(), out_vec, out_dim, 50,
				  0.5, 1, false, 1000, 250, 250);
		ofstream tsne_file(ASSET_PATH "/tsne.txt");
		for (size_t i = 0; i < numMeshes; i++)
			tsne_file << out_vec[i * out_dim + 0] << " " << out_vec[i * out_dim + 1] << endl;
		tsne_file.close();
	}
	return EXIT_SUCCESS;
}