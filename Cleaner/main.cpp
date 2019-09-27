#include <fstream>
#include <iostream>

#include <Open3D/Open3D.h>

#ifndef ASSET_PATH
#define ASSET_PATH "data"
#endif

using namespace std;
using namespace open3d;

int main(int argc, char **argv)
{
	ifstream toobig(ASSET_PATH "toobig.txt");
	string f;
	while (getline(toobig, f))
	{
		auto mesh = io::CreateMeshFromFile(f);
		auto mesh2 = mesh->RemoveDuplicatedTriangles();
		mesh2 = mesh2.RemoveDuplicatedVertices();
		mesh2 = mesh2.RemoveDegenerateTriangles();
		mesh2 = mesh2.RemoveUnreferencedVertices();
        auto mesh3 = mesh2.SimplifyQuadricDecimation(30000);
		io::WriteTriangleMesh(f.substr(0, f.size() - 4) + "_cleaned.off", *mesh3, true, true);
	}
    ifstream toosmall(ASSET_PATH "toosmall.txt");
	while (getline(toosmall, f))
	{
		auto mesh = io::CreateMeshFromFile(f);
		auto mesh2 = mesh->RemoveDuplicatedTriangles();
		mesh2 = mesh2.RemoveDuplicatedVertices();
		mesh2 = mesh2.RemoveDegenerateTriangles();
		mesh2 = mesh2.RemoveUnreferencedVertices();
        auto mesh3 = mesh2.SubdivideLoop(1);
        auto mesh4 = mesh3->SimplifyQuadricDecimation(10000);
		io::WriteTriangleMesh(f.substr(0, f.size() - 4) + "_cleaned.off", *mesh4, true, true);
	}
}