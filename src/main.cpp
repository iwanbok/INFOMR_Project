#ifndef ASSET_PATH
#define ASSET_PATH "data/"
#endif

#define ORIGINAL_DIR ASSET_PATH "LabeledDB_new"
#define PREPROCESSED_DIR ASSET_PATH "preprocessed"
#define NORMALIZED_DIR ASSET_PATH "normalized"
#define FEATURE_DIR ASSET_PATH "features"
#define INFO_DIR ASSET_PATH "info"

#define THRESHOLD 1

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

#include "Open3D/Geometry/TriangleMesh.h"
#include "Open3D/IO/TriangleMeshIO.h"

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
int total_meshes;

const int scrWidth = 1280, scrHeight = 800;
const int xStart = 280, yStart = 0;
const int pageSize = 9;
int page = 0;
vector<vector<tuple<int, filesystem::path>>> pages;

MatrixXd distances;

class CustomMenu : public igl::opengl::glfw::imgui::ImGuiMenu
{

  public:
	CustomMenu()
	{
		std::vector<std::filesystem::path> files = getAllFilesInDir(PREPROCESSED_DIR);
		total_meshes = files.size();
		std::sort(files.begin(), files.end());
		for (auto &f : files)
		{
			auto parent = f.parent_path();
			auto mesh_class = parent.string().substr(parent.parent_path().string().size() + 1);
			ccs[mesh_class]++;
		}
		for (map<string, int>::iterator it = ccs.begin(); it != ccs.end(); ++it)
			classes.push_back(it->first);
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
		}
		viewer->core_list[0].viewport = Vector4f(0, 0, xStart, scrHeight);
		viewer->data_list[0].set_visible(false, viewer->core_list[0].id);
	}

	virtual void draw_viewer_menu() override
	{
		// Draw parent menu
		// ImGuiMenu::draw_viewer_menu();
		int data_id = viewer->core_index(viewer->core().id);

		// Viewing options
		if (ImGui::CollapsingHeader("Viewing Options", ImGuiTreeNodeFlags_DefaultOpen))
		{
			if (ImGui::Button("Center object", ImVec2(-1, 0)))
			{
				viewer->core().align_camera_center(viewer->data(data_id).V,
												   viewer->data(data_id).F);
			}
			if (ImGui::Button("Snap canonical view", ImVec2(-1, 0)))
			{
				viewer->snap_to_canonical_quaternion();
			}

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
			static const char *curr_class = NULL;

			if (ImGui::BeginCombo("##combo", curr_class)) // The second parameter is the label
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

			ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
			if (ImGui::Button("Open File"))
				open_file = std::make_shared<pfd::open_file>(
					"Choose mesh", ASSET_PATH,
					vector<string>({"3D Meshes (.off, .ply, .stl)", "*.off | *.ply | *.stl"}));
			if (open_file && open_file->ready())
			{
				auto result = open_file->result();
				if (result.size())
				{
					filename = result[0];

					igl::read_triangle_mesh(filename, V, F);
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
				if (curr_class && ImGui::Button("Search"))
				{
					std::cout << "Opened file " << filename << "\n";
					auto mesh = open3d::io::CreateMeshFromFile(filename);
					PreProcess(mesh);
					Normalize(mesh);
					auto features = CalcFeatures(mesh);
					fdb.NormalizeFeatures(features);

					auto dists = fdb.CalcDistances(features);
					sort(dists.begin(), dists.end());
					pages.clear();
					page = 0;
					pages.push_back(vector<tuple<int, filesystem::path>>());
					int TP = 0, FP = 0;
					for (const auto &d : dists)
					{
						double dist;
						std::filesystem::path m;
						std::tie(dist, m) = d;
						if (dist > THRESHOLD)
							break;
						if (pages[pages.size() - 1].size() >= pageSize)
							pages.push_back(vector<tuple<int, filesystem::path>>());
						pages[pages.size() - 1].push_back(d);
						auto p = m.parent_path();
						auto m_class = p.string().substr(p.parent_path().string().size() + 1);
						if (strcmp(curr_class, m_class.c_str()))
							FP++;
						else
							TP++;
						cout << dist << ": " << m << endl;
					}
					int P = ccs[string(curr_class)];
					int N = total_meshes - P;
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
						 << "Accuracy: " << double(TP + TN) / total_meshes << endl
						 << "F1 Score: " << double(2 * TP) / double(2 * TP + FP + FN) << endl;

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
} menu;

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
		if (!filesystem::exists(FEATURE_DIR))
			filesystem::create_directories(FEATURE_DIR);
		fdb = CalculateFeaturesMeshDatabase(normalized);
	}
	else
		fdb = FeatureDatabase(FEATURE_DIR);

	distances = MatrixXd::Zero(total_meshes, total_meshes);
	if (options.find('d') == options.npos)
	{
		ofstream out(ASSET_PATH "/distances.txt");
		for (size_t i = 0; i < total_meshes; i++)
		{
			auto dists = fdb.CalcDistances(fdb.features[i]);
			for (size_t j = 0; j < total_meshes; j++)
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
		for (size_t i = 0; i < total_meshes; i++)
			for (size_t j = 0; j < total_meshes; j++)
				dist_file >> distances(i, j);
		dist_file.close();
	}

	double recall = 0, precision = 0;
	for (size_t i = 0; i < total_meshes; i++)
	{
		auto parent = fdb.meshes[i].parent_path();
		auto curr_class = parent.string().substr(parent.parent_path().string().size() + 1);
		int FP = 0, TP = 0;
		for (size_t j = 0; j < total_meshes; j++)
			if (distances(i, j) <= THRESHOLD)
			{
				auto p = fdb.meshes[j].parent_path();
				auto m_class = p.string().substr(p.parent_path().string().size() + 1);
				if (curr_class.compare(m_class))
					FP++;
				else
					TP++;
			}
		int P = ccs[curr_class];
		recall += double(TP) / P;
		precision += TP / double(TP + FP);
	}
	recall /= fdb.features.size();
	precision /= fdb.features.size();
	cout << "Recall over entire database: " << recall << endl
		 << "Precision over entire database: " << precision << endl;

	igl::readOFF(NORMALIZED_DIR "/Armadillo/281.off", V, F);

	igl::opengl::glfw::Viewer viewer;
	for (size_t i = 0; i <= pageSize; i++)
		viewer.append_mesh();
	viewer.data_list[0].set_mesh(V, F);
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
	return EXIT_SUCCESS;
}

// Sample code for multiple viewports
#if 0
igl::opengl::glfw::Viewer viewer;

viewer.load_mesh_from_file(ASSET_PATH "/bunny.off");
viewer.load_mesh_from_file(ASSET_PATH "/bunny.off");

unsigned int left_view, right_view;
int cube_id = viewer.data_list[0].id;
int sphere_id = viewer.data_list[1].id;
viewer.callback_init = [&](igl::opengl::glfw::Viewer &) {
	viewer.core().viewport = Eigen::Vector4f(0, 0, 640, 800);
	left_view = viewer.core_list[0].id;
	right_view = viewer.append_core(Eigen::Vector4f(640, 0, 640, 800));
	return false;
};

viewer.callback_key_down = [&](igl::opengl::glfw::Viewer &, unsigned int key, int mod) {
	if (key == GLFW_KEY_SPACE)
	{
		// By default, when a core is appended, all loaded meshes will be displayed in that
		// core. Displaying can be controlled by calling viewer.data().set_visible().
		viewer.data(cube_id).set_visible(false, left_view);
		viewer.data(sphere_id).set_visible(false, right_view);
	}
	return false;
};

viewer.callback_post_resize = [&](igl::opengl::glfw::Viewer &v, int w, int h) {
	v.core(left_view).viewport = Eigen::Vector4f(0, 0, w / 2, h);
	v.core(right_view).viewport = Eigen::Vector4f(w / 2, 0, w - (w / 2), h);
	return true;
};

viewer.launch();

#endif

// Code from Game Physics practical
#if 0
#include "scene.h"
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <imgui/imgui.h>
#include <iostream>

using namespace std;
using namespace Eigen;

MatrixXd V;
MatrixXi F;
igl::opengl::glfw::Viewer mgpViewer;

float currTime = 0;

// initial values
float timeStep = 0.02f;
float CRCoeff = 1.0f;
float dragCoeff = 10000;

static const char *current_item = NULL;
static bool changed_item = false;

Scene scene;

MatrixXd platV;
MatrixXi platF;
MatrixXi platT;
RowVector3d platCOM;
RowVector4d platOrientation;

void createPlatform()
{
	double platWidth = 100.0;
	platCOM << 0.0, -5.0, -0.0;
	platV.resize(9, 3);
	platF.resize(12, 3);
	platT.resize(12, 4);
	platV << -platWidth, 0.0, -platWidth, -platWidth, 0.0, platWidth, platWidth, 0.0, platWidth,
		platWidth, 0.0, -platWidth, -platWidth, -platWidth / 10.0, -platWidth, -platWidth,
		-platWidth / 10.0, platWidth, platWidth, -platWidth / 10.0, platWidth, platWidth,
		-platWidth / 10.0, -platWidth, 0.0, -platWidth / 20.0, 0.0;
	platF << 0, 1, 2, 2, 3, 0, 6, 5, 4, 4, 7, 6, 1, 0, 5, 0, 4, 5, 2, 1, 6, 1, 5, 6, 3, 2, 7, 2, 6,
		7, 0, 3, 4, 3, 7, 4;

	platOrientation << 1.0, 0.0, 0.0, 0.0;

	platT << platF, VectorXi::Constant(12, 8);
}

void updateMeshes(igl::opengl::glfw::Viewer &viewer)
{
	RowVector3d platColor;
	platColor << 0.8, 0.8, 0.8;
	RowVector3d meshColor;
	meshColor << 0.8, 0.2, 0.2;
	viewer.core().align_camera_center(scene.meshes[0].currV);
	for (int i = 0; i < scene.meshes.size(); i++)
	{
		viewer.data_list[i].clear();
		viewer.data_list[i].set_mesh(scene.meshes[i].currV, scene.meshes[i].F);
		viewer.data_list[i].set_face_based(true);
		if (current_item && i == atoi(current_item))
			viewer.data_list[i].set_colors(RowVector3d(0.2, 0.2, 0.8));
		else
			viewer.data_list[i].set_colors(meshColor);

		viewer.data_list[i].show_lines = false;
	}
	viewer.data_list[0].show_lines = false;
	viewer.data_list[0].set_colors(platColor.replicate(scene.meshes[0].F.rows(), 1));
	viewer.data_list[0].set_face_based(true);
	// viewer.core.align_camera_center(scene.meshes[0].currV);
}

bool key_down(igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifier)
{
	if (key == ' ')
	{
		viewer.core().is_animating = !viewer.core().is_animating;
		if (viewer.core().is_animating)
			cout << "Simulation running" << endl;
		else
			cout << "Simulation paused" << endl;
		return true;
	}

	if (key == 'S')
	{
		if (!viewer.core().is_animating)
		{
			scene.updateScene(timeStep, CRCoeff, dragCoeff);
			currTime += timeStep;
			updateMeshes(viewer);
			std::cout << "currTime: " << currTime << std::endl;
			return true;
		}
	}
	return false;
}

bool pre_draw(igl::opengl::glfw::Viewer &viewer)
{
	using namespace Eigen;
	using namespace std;

	if (viewer.core().is_animating)
	{
		scene.updateScene(timeStep, CRCoeff, dragCoeff);
		currTime += timeStep;
		// cout <<"currTime: "<<currTime<<endl;
		updateMeshes(viewer);
	}
	else if (changed_item)
	{
		updateMeshes(viewer);
		changed_item = false;
	}

	return false;
}

class CustomMenu : public igl::opengl::glfw::imgui::ImGuiMenu
{
	vector<string> mesh_labels;

  public:
	CustomMenu()
	{
		for (int i = 0; i < scene.meshes.size(); i++)
			mesh_labels.push_back(to_string(i));
	}

	virtual void draw_viewer_menu() override
	{
		// Draw parent menu
		ImGuiMenu::draw_viewer_menu();

		// Add new group
		if (ImGui::CollapsingHeader("Algorithm Options", ImGuiTreeNodeFlags_DefaultOpen))
		{
			ImGui::InputFloat("CR Coeff", &CRCoeff, 0, 0, 3);
			ImGui::InputFloat("Drag Coeff", &dragCoeff, 0, 0, 3);

			if (ImGui::InputFloat("Time Step", &timeStep))
			{
				mgpViewer.core().animation_max_fps = (((int)1.0 / timeStep));
			}
		}

		if (ImGui::CollapsingHeader("Changing velocities", ImGuiTreeNodeFlags_DefaultOpen))
		{

			if (ImGui::BeginCombo("Object id", current_item))
			{
				for (int n = 0; n < mesh_labels.size(); n++)
				{
					bool is_selected =
						(current_item ==
						 mesh_labels[n].c_str()); // You can store your selection however you
												  // want, outside or inside your objects
					if (ImGui::Selectable(mesh_labels[n].c_str(), is_selected))
						current_item = mesh_labels[n].c_str();
					if (is_selected)
						ImGui::SetItemDefaultFocus(); // You may set the initial focus when opening
													  // the combo (scrolling + for keyboard
													  // navigation support)
				}
				changed_item = true;
				ImGui::EndCombo();
			}

			if (current_item)
			{
				ImGui::InputDouble("Linear v x",
								   &scene.meshes[atoi(current_item)].comVelocity.data()[0], 0, 0,
								   "%.3f");
				ImGui::InputDouble("Linear v y",
								   &scene.meshes[atoi(current_item)].comVelocity.data()[1], 0, 0,
								   "%.3f");
				ImGui::InputDouble("Linear v z",
								   &scene.meshes[atoi(current_item)].comVelocity.data()[2], 0, 0,
								   "%.3f");
				ImGui::InputDouble("Angluar v x",
								   &scene.meshes[atoi(current_item)].angVelocity.data()[0], 0, 0,
								   "%.3f");
				ImGui::InputDouble("Angluar v y",
								   &scene.meshes[atoi(current_item)].angVelocity.data()[1], 0, 0,
								   "%.3f");
				ImGui::InputDouble("Angluar v z",
								   &scene.meshes[atoi(current_item)].angVelocity.data()[2], 0, 0,
								   "%.3f");
			}
		}
	}
};

int main(int argc, char *argv[])
{
	using namespace Eigen;
	using namespace std;

	// Load scene
	if (argc < 3)
	{
		cout << "Please provide path (argument 1 and name of scene file (argument 2)!" << endl;
		return 0;
	}
	cout << "scene file: " << std::string(argv[2]) << endl;
	// create platform
	createPlatform();
	scene.addMesh(platV, platF, platT, 10000.0, true, platCOM, platOrientation);

	// load scene from file
	scene.loadScene(std::string(argv[1]), std::string(argv[2]));

	scene.updateScene(0.0, CRCoeff, dragCoeff);

	// Viewer Settings
	for (int i = 0; i < scene.meshes.size(); i++)
	{
		if (i != 0)
			mgpViewer.append_mesh();
		// mgpViewer.data_list[i].set_mesh(scene.meshes[i].currV, scene.meshes[i].F);
	}
	// mgpViewer.core.align_camera_center(scene.meshes[0].currV);

	mgpViewer.callback_pre_draw = &pre_draw;
	mgpViewer.callback_key_down = &key_down;
	mgpViewer.core().is_animating = false;
	mgpViewer.core().animation_max_fps = 50.;
	updateMeshes(mgpViewer);
	CustomMenu menu;
	mgpViewer.plugins.push_back(&menu);

	cout << "Press [space] to toggle continuous simulation" << endl;
	cout << "Press 'S' to advance time step-by-step" << endl;

	mgpViewer.launch();
}
#endif