#pragma once

#include <filesystem>
#include <vector>
#include <fstream>

#include "Open3D/IO/TriangleMeshIO.h"

#include "Utils.hpp"

struct Features
{
    /* TODO data */

    Features() {}
    Features(const std::filesystem::path &file)
    {
        /* TODO set data */
    }

    void WriteToFile(const std::filesystem::path &file) const
    {
        std::ofstream out(file);
        /* TODO write data */
        out.close();
    }
};

Features CalcFeatures(const open3d::geometry::TriangleMesh &mesh)
{
    Features features;
    // TODO CALC Features
    return features;
}

void CalculateFeaturesMeshDatabase(const std::vector<std::filesystem::path> &files)
{
    for (auto &f : files)
    {
        auto mesh = open3d::io::CreateMeshFromFile(f);
        auto features = CalcFeatures(*mesh);
        features.WriteToFile(replaceDir(f, FEATURE_DIR));
    }    
}