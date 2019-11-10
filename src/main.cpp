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
#include <sstream>
#include <string>
#include <vector>

#include <ANN/ANN.h>

// #include <Python.h>
// #include <matplotlib/matplotlibcpp.h>
// namespace plt = matplotlibcpp;

#include "Open3D/Geometry/TriangleMesh.h"
#include "Open3D/IO/TriangleMeshIO.h"

#include "tsne/tsne.h"

#include "defines.h"
#include "FeatureDatabase.hpp"
#include "Normalize.hpp"
#include "PreProcess.hpp"

using namespace Eigen;
using namespace std;

FeatureDatabase fdb;
MatrixXd V;
MatrixXi F;
vector<string> classes;
map<string, size_t> ccs;

const int scrWidth = 1280, scrHeight = 800;
const int xStart = 280, yStart = 0;
const int pageSize = 9;
int page = 0;
vector<vector<pair<double, filesystem::path>>> pages;

double r_ann = 1.0;
int k_ann = 20;
double r_us = 0.45;
int k_us = 20;
std::vector<int> weights(10);

/// Custom ImGui menu for the MR application.
class CustomMenu : public igl::opengl::glfw::imgui::ImGuiMenu
{
  public:
	ANNkd_tree f_tree;
	int TP = 0, FP = 0;
	const char *curr_class = NULL;
	string filename = "";
	const char *algs[4] = {"K-nearest ANN", "R-nearest ANN", "K-nearest Custom",
						   "R-nearest Custom"};
	const char *curr_alg = algs[0];

	CustomMenu(double *pa, int n) : f_tree(pa, n, Features::size())
	{
	}

	/// Reset the camera for given viewer core to original position.
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

	/// Setup the viewer to display the current page of results.
	void set_result_meshes()
	{
		for (size_t i = 0; i < pageSize; i++)
		{
			viewer->data_list[i + 1].clear();
			if (i >= pages[page].size())
				continue;
			viewer->data_list[i + 1].set_visible(true, viewer->core_list[i + 1].id);
			igl::read_triangle_mesh(pages[page][i].second.string(), V, F);
			viewer->data_list[i + 1].set_mesh(V, F);

			// Resetting camera parameters
			reset_camera(viewer->core_list[i + 1]);
		}
		viewer->core_list[0].viewport = Vector4f(0, 0, xStart, scrHeight);
		viewer->data_list[0].set_visible(false, viewer->core_list[0].id);
	}

	/// Setup the pages vector and increment correct statistic(FP or TP) if curr_class has a value.
	void setup_pages_and_stats(const pair<double, filesystem::path> &d)
	{
		if (pages[pages.size() - 1].size() >= pageSize)
			pages.push_back(vector<pair<double, filesystem::path>>());
		pages[pages.size() - 1].push_back(d);
		if (curr_class)
		{
			auto m_class = d.second.parent_path().filename().string();
			if (m_class.compare(curr_class))
				FP++;
			else
				TP++;
		}
		cout << d.first << ": " << d.second << endl;
	}

	/// Overide menu drawer to add labels in result viewer.
	virtual void draw_menu() override
	{
		// Text labels
		draw_labels_window();
		if (page < pages.size())
			draw_custom_lables();

		// Viewer settings
		if (callback_draw_viewer_window)
		{
			callback_draw_viewer_window();
		}
		else
		{
			draw_viewer_window();
		}

		// Other windows
		if (callback_draw_custom_window)
		{
			callback_draw_custom_window();
		}
		else
		{
			draw_custom_window();
		}
	}

	/// Create ImGui window with the labels for the currently selected page.
	void draw_custom_lables()
	{
		// Create window
		ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiSetCond_Always);
		ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize, ImGuiSetCond_Always);
		bool visible = true;
		ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0, 0, 0, 0));
		ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
		ImGui::Begin("CustomLabels", &visible,
					 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
						 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
						 ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoCollapse |
						 ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoInputs);
		// Add labels to draw list
		ImDrawList *drawList = ImGui::GetWindowDrawList();
		for (size_t i = 0; i < pages[page].size(); i++)
		{
			ostringstream strs;
			strs << "distance: " << pages[page][i].first;
			string str = strs.str();
			const auto &viewport = viewer->core_list[i + 1].viewport;
			drawList->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 1.2f,
							  ImVec2(viewport[0] + 10, scrHeight - viewport[1] - viewport[3] + 10),
							  ImGui::GetColorU32(ImVec4(1, 1, 1, 1)), &str[0],
							  &str[0] + str.size());
			str = "mesh: " + (pages[page][i].second.parent_path().filename() /
							  pages[page][i].second.filename())
								 .string();
			drawList->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 1.2f,
							  ImVec2(viewport[0] + 10, scrHeight - viewport[1] - viewport[3] +
														   ImGui::GetTextLineHeight() + 10),
							  ImGui::GetColorU32(ImVec4(1, 1, 1, 1)), &str[0],
							  &str[0] + str.size());
		}
		// End drawing of window
		ImGui::End();
		ImGui::PopStyleColor();
		ImGui::PopStyleVar();
	}

	/// Viewing options submenu
	void ViewingOptions()
	{
		if (ImGui::CollapsingHeader("Viewing Options", ImGuiTreeNodeFlags_DefaultOpen))
		{
			// Snap to canonical
			if (ImGui::Button("Snap canonical view", ImVec2(-1, 0)))
				viewer->snap_to_canonical_quaternion();
			// Reset camera
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
	}

	/// Drawing options submenu
	void DrawingOptions()
	{
		if (ImGui::CollapsingHeader("Draw Options", ImGuiTreeNodeFlags_DefaultOpen))
		{
			// Face based rendering
			static bool face_based = false;
			if (ImGui::Checkbox("Face-based", &face_based))
			{
				for (auto &d : viewer->data_list)
				{
					d.face_based = face_based;
					d.dirty = igl::opengl::MeshGL::DIRTY_ALL;
				}
			}

			// Shininess
			ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.3f);
			static float shininess = 50.f;
			if (ImGui::DragFloat("Shininess", &shininess, 0.05f, 0.0f, 100.0f))
				for (auto &d : viewer->data_list)
					d.shininess = shininess;
			ImGui::PopItemWidth();

			// Wireframe drawing toggle
			static bool wireframe = true;
			if (ImGui::Checkbox("Wireframe", &wireframe))
				for (size_t i = 0; i <= pageSize; i++)
					viewer->core_list[i].set(viewer->data_list[i].show_lines, wireframe);

			// Filling drawing toggle
			static bool fill = true;
			if (ImGui::Checkbox("Fill", &fill))
				for (size_t i = 0; i <= pageSize; i++)
					viewer->core_list[i].set(viewer->data_list[i].show_faces, fill);
		}
	}

	void Search()
	{
		if (ImGui::Button("Search"))
		{
			std::cout << "Opened file " << filename << "\n";
			auto mesh = open3d::io::CreateMeshFromFile(filename);
			PreProcess(mesh);
			Normalize(mesh);
			Features features(mesh);
			fdb.NormalizeFeatures(features);
			features.WeightFeatures(weights);

			pages.push_back(vector<pair<double, filesystem::path>>());
			TP = 0, FP = 0;
			if (curr_alg == algs[0])
			{
				vector<int> nn_idx(k_ann);
				vector<double> dd(k_ann);
				f_tree.annkSearch(features.data(), k_ann, nn_idx.data(), dd.data());
				for (size_t i = 0; i < k_ann; i++)
					setup_pages_and_stats(make_pair(dd[i], fdb.meshes[nn_idx[i]]));
			}
			else if (curr_alg == algs[1])
			{
				vector<int> nn_idx(numMeshes);
				vector<double> dd(numMeshes);
				f_tree.annkSearch(features.data(), int(numMeshes), nn_idx.data(), dd.data());
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
				size_t P = ccs[string(curr_class)];
				size_t N = numMeshes - P;
				size_t TN = N - FP;
				size_t FN = P - TP;

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

	/// Search options submenu
	void SearchOptions()
	{
		if (ImGui::CollapsingHeader("Search Options", ImGuiTreeNodeFlags_DefaultOpen))
		{
			static shared_ptr<pfd::open_file> open_file;

			// Class picker
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

			// Algorithm picker
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

			// Algorithm parameter
			const double minDrag = 0.1, maxDrag = 10.0;
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

			// File dialog
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
					reset_camera(viewer->core_list[0]);
					for (size_t i = 0; i < pageSize; i++)
						viewer->data_list[i + 1].set_visible(false, viewer->core_list[i + 1].id);
				}
				open_file = nullptr;
			}
			ImGui::PopItemFlag();

			if (filename.size())
			{
				// Selected file textfield
				filesystem::path filePath(filename);
				ImGui::SameLine();
				ImGui::Text(
					(filePath.parent_path().filename() / filePath.filename()).string().c_str());

				// Search button and logic
				Search();
			}

			// Page switcher
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

	/// Main menu and only menu user can interact with.
	virtual void draw_viewer_menu() override
	{
		ViewingOptions();

		DrawingOptions();

		SearchOptions();
	}
};

int main(int argc, char *argv[])
{
	string options = GetCmdOptions(argc, argv);
	if (options.find('h') != options.npos)
	{
		cout << "Command line options:" << endl
			 << "\t-h: This help menu" << endl
			 << "\t-p: Preprocess Database" << endl
			 << "\t-n: Normalize Meshes" << endl
			 << "\t-f: Calculate database feautures" << endl
			 << "\t-s: Calculate database retrieval performance statistics" << endl
			 << "\t-t: Perform T-SNE calculation" << endl;
		return EXIT_SUCCESS;
	}

	// Preprocess
	if (options.find('p') != options.npos)
	{
		vector<filesystem::path> originals = getAllFilesInDir(ORIGINAL_DIR);
		if (!filesystem::exists(PREPROCESSED_DIR))
			filesystem::create_directories(PREPROCESSED_DIR);
		PreProcessMeshDatabase(originals);
	}

	// Normalize
	if (options.find('n') != options.npos)
	{
		vector<filesystem::path> preprossed = getAllFilesInDir(PREPROCESSED_DIR);
		if (!filesystem::exists(NORMALIZED_DIR))
			filesystem::create_directories(NORMALIZED_DIR);
		NormalizeMeshDataBase(preprossed);
	}

	// Gather feature database
	if (options.find('f') != options.npos)
	{
		if (options.find('n') == options.npos)
			CalcMeshStatistics(getAllFilesInDir(NORMALIZED_DIR));
		vector<filesystem::path> normalized = getAllFilesInDir(NORMALIZED_DIR);
		sort(normalized.begin(), normalized.end(), greater<filesystem::path>());
		if (!filesystem::exists(FEATURE_DIR))
			filesystem::create_directories(FEATURE_DIR);
		fdb = FeatureDatabase(normalized);
	}
	else
		fdb = FeatureDatabase(FEATURE_DIR);

	// Gather class stats
	std::vector<std::filesystem::path> files = getAllFilesInDir(PREPROCESSED_DIR);
	for (auto &f : files)
		ccs[f.parent_path().filename().string()]++;
	for (auto it = ccs.begin(); it != ccs.end(); ++it)
		classes.push_back(it->first);

	// Gather weights (EDIT CODE MANUALLY)
	if (options.find('w') != options.npos)
	{
		double best_prec = 0;
		const int k = 20;
		int i1 = 1, i3 = 1, i4 = 1, i6 = 1, i9 = 1;
		for (int i2 = 4; i2 < 10; i2++)
			for (int i5 = 2; i5 < 10; i5++)
				for (int i7 = 2; i7 < 10; i7++)
					for (int i8 = 3; i8 < 10; i8++)
						for (int i10 = 2; i10 < 10; i10++)
						{
							vector<int> curr_weights({i1, i2, i3, i4, i5, i6, i7, i8, i9, i10});
							auto w_feat = fdb.GetWeightedFeatures(curr_weights);
							CustomMenu menu(w_feat.data(), int(numMeshes));
							double precision = 0;
							for (size_t i = 0; i < numMeshes; i++)
							{
								auto curr_class = fdb.meshes[i].parent_path().filename().string();
								int FP = 0, TP = 0;
								vector<int> nn_idx(k);
								vector<double> dd(k);
								menu.f_tree.annkSearch(&w_feat[i * Features::size()], k,
													   nn_idx.data(), dd.data());
								for (int id : nn_idx)
								{
									auto m_class = fdb.meshes[id].parent_path().filename().string();
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
								weights = curr_weights;
							}
						}
		ofstream weight_file(WEIGHTS_FILE);
		for (size_t i = 0; i < 10; i++)
			weight_file << weights[i] << endl;
		weight_file.close();
	}
	else
	{
		ifstream weight_file(WEIGHTS_FILE);
		for (size_t i = 0; i < 10; i++)
			weight_file >> weights[i];
		weight_file.close();
	}

	// Weight database and setup menu
	fdb.WeightFeatures(weights);
	auto feats = fdb.GetFeatureVectors();
	CustomMenu menu(feats.data(), int(fdb.features.size()));

	// Calculate statistics
	if (options.find('a') != options.npos || options.find('c') != options.npos)
	{
		map<string, double> classPerf;

		for (size_t i = 0; i < numMeshes; i++)
		{
			auto curr_class = fdb.meshes[i].parent_path().filename().string();
			int FP = 0, TP = 0, k = 20;
			if (options.find('a') != options.npos)
			{
				vector<int> nn_idx(k);
				vector<double> dd(k);
				menu.f_tree.annkSearch(&feats[i * Features::size()], k, nn_idx.data(), dd.data());
				for (int id : nn_idx)
				{
					auto m_class = fdb.meshes[id].parent_path().filename().string();
					if (curr_class.compare(m_class))
						FP++;
					else
						TP++;
				}
			}
			else
			{
				auto dists = fdb.CalcDistances(fdb.features[i]);
				sort(dists.begin(), dists.end());
				for (size_t i = 0; i < k; i++)
				{
					auto m_class = dists[i].second.parent_path().filename().string();
					if (curr_class.compare(m_class))
						FP++;
					else
						TP++;
				}
			}
			classPerf[curr_class] += TP / double(TP + FP);
		}

		// Average stats over classes and total
		double precision = 0;
		ofstream perf_file(ASSET_PATH "/perf.txt");
		if (options.find('a') != options.npos)
		{
			cout << "Calculated precision with ANN" << endl;
			perf_file << "ANN Statistics" << endl;
		}
		else
		{
			cout << "Calculated precision with Custom metrics" << endl;
			perf_file << "Custom metric Statistics" << endl;
		}
		perf_file << "Class Precision" << endl;
		cout << "Class Precision" << endl;
		for (const auto &cs : ccs)
		{
			precision += classPerf[cs.first];
			classPerf[cs.first] /= cs.second;
			perf_file << cs.first << " " << classPerf[cs.first] << endl;
			cout << cs.first << " " << classPerf[cs.first] << endl;
		}
		precision /= fdb.features.size();
		perf_file << "total " << precision << endl;
		perf_file.close();
		cout << "Precision over entire database: " << precision << endl;
	}

	// Setup viewer
	igl::opengl::glfw::Viewer viewer;
	for (size_t i = 0; i <= pageSize; i++)
		viewer.append_mesh();
	viewer.callback_init = [&](igl::opengl::glfw::Viewer &) {
		viewer.core().viewport = Vector4f(0, 0, scrWidth, scrHeight);
		const int xCount = int(ceil(pageSize / 3.0));
		const int yCount = 3;
		assert(xCount * yCount <= 31);
		const float cellWidth = float((scrWidth - xStart) / xCount);
		const float cellHeight = float((scrHeight - yStart) / yCount);
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

	viewer.plugins.push_back(&menu);
	viewer.launch();

	// T-SNE calculation
	if (options.find('t') != options.npos)
	{
		auto why_you_edit_pointer = fdb.GetFeatureVectors();
		size_t out_dim = 2;
		double *out_vec = (double *)malloc(numMeshes * out_dim * sizeof(double));
		double *costs = (double *)calloc(numMeshes, sizeof(double));
		TSNE::run(why_you_edit_pointer.data(), int(numMeshes), Features::size(), out_vec,
				  int(out_dim), 50, 0.5, 1, false, 1000, 250, 250);
		ofstream tsne_file(ASSET_PATH "/tsne.txt");
		// vector<double> X(numMeshes);
		// vector<double> Y(numMeshes);
		// vector<double> C(numMeshes);
		for (size_t i = 0; i < numMeshes; i++)
		{
			tsne_file << out_vec[i * out_dim + 0] << " " << out_vec[i * out_dim + 1] << endl;
			// X[i] = out_vec[i * out_dim + 0];
			// Y[i] = out_vec[i * out_dim + 1];
			// C[i] = (i / 20) / 20.f;
		}
		tsne_file.close();
		// Failed python plot in c++
		// try
		// {
		// 	plt::scatter(X, Y);
		// 	plt::show();
		// }
		// catch (const std::exception &e)
		// {
		// 	std::cerr << e.what() << endl
		// 			  << "Install Python with MatPlotLib or if that failes use:" << endl
		// 			  << "\t`python plot.py`" << endl
		// 			  << "to view the plot of the t-sne." << endl;
		// }
	}
	return EXIT_SUCCESS;
}