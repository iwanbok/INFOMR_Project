#ifndef ASSET_PATH
#define ASSET_PATH "data"
#endif

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/read_triangle_mesh.h>
#include <imgui/imgui.h>

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "Open3D/Geometry/TriangleMesh.h"
#include "Open3D/IO/TriangleMeshIO.h"

#include "PreProcess.hpp"

using namespace Eigen;
using namespace std;
namespace filesys = std::filesystem;

MatrixXd V;
MatrixXi F;

class CustomMenu : public igl::opengl::glfw::imgui::ImGuiMenu
{
  public:
	CustomMenu()
	{
	}

	virtual void draw_viewer_menu() override
	{
		// Draw parent menu
		ImGuiMenu::draw_viewer_menu();

		// Add new group
		if (ImGui::CollapsingHeader("Search Options", ImGuiTreeNodeFlags_DefaultOpen))
		{
		}
	}
} menu;

bool key_down(igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifier)
{
	return false;
}

// Based on
// https://thispointer.com/c-get-the-list-of-all-files-in-a-given-directory-and-its-sub-directories-using-boost-c17/
/*
 * Get the list of all files in given directory and its sub directories.
 *
 * Arguments
 * 	dirPath : Path of directory to be traversed
 * 	dirSkipList : List of folder names to be skipped
 *
 * Returns:
 * 	vector containing paths of all the files in given directory and its sub directories
 *
 */
vector<string> getAllFilesInDir(const string &dirPath, const vector<string> dirSkipList = {})
{

	// Create a vector of string
	std::vector<string> listOfFiles;
	try
	{
		// Check if given path exists and points to a directory
		if (filesys::exists(dirPath) && filesys::is_directory(dirPath))
		{
			// Create a Recursive Directory Iterator object and points to the starting of directory
			filesys::recursive_directory_iterator iter(dirPath);

			// Create a Recursive Directory Iterator object pointing to end.
			filesys::recursive_directory_iterator end;

			// Iterate till end
			while (iter != end)
			{
				// Check if current entry is a directory and if exists in skip list
				if (filesys::is_directory(iter->path()) &&
					(std::find(dirSkipList.begin(), dirSkipList.end(), iter->path().filename()) !=
					 dirSkipList.end()))
					// Skip the iteration of current directory pointed by iterator
					iter.disable_recursion_pending();
				else if (!filesys::is_directory(iter->path()))
					// Add the name in vector
					listOfFiles.push_back(iter->path().string());

				error_code ec;
				// Increment the iterator to point to next entry in recursive iteration
				iter.increment(ec);
				if (ec)
				{
					cerr << "Error While Accessing : " << iter->path().string()
						 << " :: " << ec.message() << endl;
				}
			}
		}
	}
	catch (system_error &e)
	{
		cerr << "Exception :: " << e.what();
	}
	return listOfFiles;
}

int main(int argc, char *argv[])
{
	vector<string> files = getAllFilesInDir(ASSET_PATH "LabeledDB_new/");
	preProcessMeshDatabase(files);

	// Load a mesh in OFF format
	igl::readOFF(ASSET_PATH "bunny.off", V, F);

	igl::opengl::glfw::Viewer viewer;

	viewer.data().set_mesh(V, F);

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