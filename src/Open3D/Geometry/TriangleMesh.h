// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "MeshBase.h"
#include "../Utility/Helper.h"

namespace open3d {
namespace geometry {

class TriangleMesh : public MeshBase {
public:
    TriangleMesh() : MeshBase(Geometry::GeometryType::TriangleMesh) {}
    ~TriangleMesh() override {}

public:
    virtual TriangleMesh &Clear() override;
    virtual TriangleMesh &Transform(
            const Eigen::Matrix4d &transformation) override;
    virtual TriangleMesh &Rotate(
            const Eigen::Vector3d &rotation,
            bool center = true,
            RotationType type = RotationType::XYZ) override;

public:
    TriangleMesh &operator+=(const TriangleMesh &mesh);
    TriangleMesh operator+(const TriangleMesh &mesh) const;

    bool HasTriangles() const {
        return vertices_.size() > 0 && triangles_.size() > 0;
    }

    bool HasTriangleNormals() const {
        return HasTriangles() && triangles_.size() == triangle_normals_.size();
    }

    bool HasAdjacencyList() const {
        return vertices_.size() > 0 &&
               adjacency_list_.size() == vertices_.size();
    }

    bool HasTriangleUvs() const {
        return HasTriangles() && triangle_uvs_.size() == 3 * triangles_.size();
    }

    TriangleMesh &NormalizeNormals() {
        MeshBase::NormalizeNormals();
        for (size_t i = 0; i < triangle_normals_.size(); i++) {
            triangle_normals_[i].normalize();
            if (std::isnan(triangle_normals_[i](0))) {
                triangle_normals_[i] = Eigen::Vector3d(0.0, 0.0, 1.0);
            }
        }
        return *this;
    }

    /// Function to compute triangle normals, usually called before rendering
    TriangleMesh &ComputeTriangleNormals(bool normalized = true);

    /// Function to compute vertex normals, usually called before rendering
    TriangleMesh &ComputeVertexNormals(bool normalized = true);

    /// Function to compute adjacency list, call before adjacency list is needed
    TriangleMesh &ComputeAdjacencyList();

    /// Function that removes duplicated verties, i.e., vertices that have
    /// identical coordinates.
    TriangleMesh &RemoveDuplicatedVertices();

    /// Function that removes duplicated triangles, i.e., removes triangles
    /// that reference the same three vertices, independent of their order.
    TriangleMesh &RemoveDuplicatedTriangles();

    /// This function removes vertices from the triangle mesh that are not
    /// referenced in any triangle of the mesh.
    TriangleMesh &RemoveUnreferencedVertices();

    /// Function that removes degenerate triangles, i.e., triangles that
    /// reference a single vertex multiple times in a single triangle.
    /// They are usually the product of removing duplicated vertices.
    TriangleMesh &RemoveDegenerateTriangles();

    /// Function that removes all non-manifold edges, by successively deleting
    /// triangles with the smallest surface area adjacent to the non-manifold
    /// edge until the number of adjacent triangles to the edge is `<= 2`.
    TriangleMesh &RemoveNonManifoldEdges();

    /// Function to sharpen triangle mesh. The output value ($v_o$) is the
    /// input value ($v_i$) plus \param strength times the input value minus
    /// the sum of he adjacent values.
    /// $v_o = v_i x strength (v_i * |N| - \sum_{n \in N} v_n)$.
    /// \param number_of_iterations defines the number of repetitions
    /// of this operation.
    std::shared_ptr<TriangleMesh> FilterSharpen(
            int number_of_iterations,
            double strength,
            FilterScope scope = FilterScope::All) const;

    /// Function to smooth triangle mesh with simple neighbour average.
    /// $v_o = \frac{v_i + \sum_{n \in N} v_n)}{|N| + 1}$, with $v_i$
    /// being the input value, $v_o$ the output value, and $N$ is the
    /// set of adjacent neighbours.
    /// \param number_of_iterations defines the number of repetitions
    /// of this operation.
    std::shared_ptr<TriangleMesh> FilterSmoothSimple(
            int number_of_iterations,
            FilterScope scope = FilterScope::All) const;

    /// Function to smooth triangle mesh using Laplacian.
    /// $v_o = v_i \cdot \lambda (sum_{n \in N} w_n v_n - v_i)$,
    /// with $v_i$ being the input value, $v_o$ the output value, $N$ is the
    /// set of adjacent neighbours, $w_n$ is the weighting of the neighbour
    /// based on the inverse distance (closer neighbours have higher weight),
    /// and \param lambda is the smoothing parameter.
    /// \param number_of_iterations defines the number of repetitions
    /// of this operation.
    std::shared_ptr<TriangleMesh> FilterSmoothLaplacian(
            int number_of_iterations,
            double lambda,
            FilterScope scope = FilterScope::All) const;

    /// Function to smooth triangle mesh using method of Taubin,
    /// "Curve and Surface Smoothing Without Shrinkage", 1995.
    /// Applies in each iteration two times FilterSmoothLaplacian, first
    /// with \param lambda and second with \param mu as smoothing parameter.
    /// This method avoids shrinkage of the triangle mesh.
    /// \param number_of_iterations defines the number of repetitions
    /// of this operation.
    std::shared_ptr<TriangleMesh> FilterSmoothTaubin(
            int number_of_iterations,
            double lambda = 0.5,
            double mu = -0.53,
            FilterScope scope = FilterScope::All) const;

    /// Function that computes the Euler-Poincaré characteristic, i.e.,
    /// V + F - E, where V is the number of vertices, F is the number
    /// of triangles, and E is the number of edges.
    int EulerPoincareCharacteristic() const;

    /// Function that returns the non-manifold edges of the triangle mesh.
    /// If \param allow_boundary_edges is set to false, than also boundary
    /// edges are returned
    std::vector<Eigen::Vector2i> GetNonManifoldEdges(
            bool allow_boundary_edges = true) const;

    /// Function that checks if the given triangle mesh is edge-manifold.
    /// A mesh is edge­-manifold if each edge is bounding either one or two
    /// triangles. If allow_boundary_edges is set to false, then this function
    /// returns false if there exists boundary edges.
    bool IsEdgeManifold(bool allow_boundary_edges = true) const;

    /// Function that returns a list of non-manifold vertex indices.
    /// A vertex is manifold if its star is edge‐manifold and edge‐connected.
    /// (Two or more faces connected only by a vertex and not by an edge.)
    std::vector<int> GetNonManifoldVertices() const;

    /// Function that checks if all vertices in the triangle mesh are manifold.
    /// A vertex is manifold if its star is edge‐manifold and edge‐connected.
    /// (Two or more faces connected only by a vertex and not by an edge.)
    bool IsVertexManifold() const;

    /// Function that tests if the given triangle mesh is orientable, i.e.
    /// the triangles can oriented in such a way that all normals point
    /// towards the outside.
    bool IsOrientable() const;

    /// If the mesh is orientable then this function rearranges the
    /// triangles such that all normals point towards the
    /// outside/inside.
    bool OrientTriangles();

    /// Function that returns a map from edges (vertex0, vertex1) to the
    /// triangle indices the given edge belongs to.
    std::unordered_map<Eigen::Vector2i,
                       std::vector<int>,
                       utility::hash_eigen::hash<Eigen::Vector2i>>
    GetEdgeToTrianglesMap() const;

    /// Function that computes the area of a mesh triangle
    static double ComputeTriangleArea(const Eigen::Vector3d &p0,
                                      const Eigen::Vector3d &p1,
                                      const Eigen::Vector3d &p2);

    /// Function that computes the area of a mesh triangle identified by the
    /// triangle index
    double GetTriangleArea(size_t triangle_idx) const;

    /// Function that computes the surface area of the mesh, i.e. the sum of
    /// the individual triangle surfaces.
    double GetSurfaceArea() const;

    /// Function that computes the surface area of the mesh, i.e. the sum of
    /// the individual triangle surfaces.
    double GetSurfaceArea(std::vector<double> &triangle_areas) const;

    /// Function that computes the plane equation from the three points.
    /// If the three points are co-linear, then this function returns the
    /// invalid plane (0, 0, 0, 0).
    static Eigen::Vector4d ComputeTrianglePlane(const Eigen::Vector3d &p0,
                                                const Eigen::Vector3d &p1,
                                                const Eigen::Vector3d &p2);

    /// Function that computes the plane equation of a mesh triangle identified
    /// by the triangle index.
    Eigen::Vector4d GetTrianglePlane(size_t triangle_idx) const;

    /// Function to subdivide triangle mesh using the simple midpoint algorithm.
    /// Each triangle is subdivided into four triangles per iteration and the
    /// new vertices lie on the midpoint of the triangle edges.
    std::shared_ptr<TriangleMesh> SubdivideMidpoint(
            int number_of_iterations) const;

    /// Function to subdivide triangle mesh using Loop's scheme.
    /// Cf. Charles T. Loop, "Smooth subdivision surfaces based on triangles",
    /// 1987. Each triangle is subdivided into four triangles per iteration.
    std::shared_ptr<TriangleMesh> SubdivideLoop(int number_of_iterations) const;

    /// Function to simplify mesh using Vertex Clustering.
    /// The result can be a non-manifold mesh.
    std::shared_ptr<TriangleMesh> SimplifyVertexClustering(
            double voxel_size,
            SimplificationContraction contraction =
                    SimplificationContraction::Average) const;

    /// Function to simplify mesh using Quadric Error Metric Decimation by
    /// Garland and Heckbert.
    std::shared_ptr<TriangleMesh> SimplifyQuadricDecimation(
            int target_number_of_triangles) const;

    /// Factory function to create a tetrahedron mesh (trianglemeshfactory.cpp).
    /// the mesh centroid will be at (0,0,0) and \param radius defines the
    /// distance from the center to the mesh vertices.
    static std::shared_ptr<TriangleMesh> CreateTetrahedron(double radius = 1.0);

    /// Factory function to create an octahedron mesh (trianglemeshfactory.cpp).
    /// the mesh centroid will be at (0,0,0) and \param radius defines the
    /// distance from the center to the mesh vertices.
    static std::shared_ptr<TriangleMesh> CreateOctahedron(double radius = 1.0);

    /// Factory function to create an icosahedron mesh
    /// (trianglemeshfactory.cpp). the mesh centroid will be at (0,0,0) and
    /// \param radius defines the distance from the center to the mesh vertices.
    static std::shared_ptr<TriangleMesh> CreateIcosahedron(double radius = 1.0);

    /// Factory function to create a box mesh (TriangleMeshFactory.cpp)
    /// The left bottom corner on the front will be placed at (0, 0, 0).
    /// The \param width is x-directional length, and \param height and \param
    /// depth are y- and z-directional lengths respectively.
    static std::shared_ptr<TriangleMesh> CreateBox(double width = 1.0,
                                                   double height = 1.0,
                                                   double depth = 1.0);

    /// Factory function to create a sphere mesh (TriangleMeshFactory.cpp)
    /// The sphere with \param radius will be centered at (0, 0, 0).
    /// Its axis is aligned with z-axis.
    /// The longitudes will be split into \param resolution segments.
    /// The latitudes will be split into \param resolution * 2 segments.
    static std::shared_ptr<TriangleMesh> CreateSphere(double radius = 1.0,
                                                      int resolution = 20);

    /// Factory function to create a cylinder mesh (TriangleMeshFactory.cpp)
    /// The axis of the cylinder will be from (0, 0, -height/2) to (0, 0,
    /// height/2). The circle with \param radius will be split into \param
    /// resolution segments. The \param height will be split into \param split
    /// segments.
    static std::shared_ptr<TriangleMesh> CreateCylinder(double radius = 1.0,
                                                        double height = 2.0,
                                                        int resolution = 20,
                                                        int split = 4);

    /// Factory function to create a cone mesh (TriangleMeshFactory.cpp)
    /// The axis of the cone will be from (0, 0, 0) to (0, 0, \param height).
    /// The circle with \param radius will be split into \param resolution
    /// segments. The height will be split into \param split segments.
    static std::shared_ptr<TriangleMesh> CreateCone(double radius = 1.0,
                                                    double height = 2.0,
                                                    int resolution = 20,
                                                    int split = 1);

    /// Factory function to create a torus mesh (TriangleMeshFactory.cpp)
    /// The torus will be centered at (0, 0, 0) and a radius of \param
    /// torus_radius. The tube of the torus will have a radius of \param
    /// tube_radius. The number of segments in radial and tubular direction are
    /// \param radial_resolution and \param tubular_resolution respectively.
    static std::shared_ptr<TriangleMesh> CreateTorus(
            double torus_radius = 1.0,
            double tube_radius = 0.5,
            int radial_resolution = 30,
            int tubular_resolution = 20);

    /// Factory function to create an arrow mesh (TriangleMeshFactory.cpp)
    /// The axis of the cone with \param cone_radius will be along the z-axis.
    /// The cylinder with \param cylinder_radius is from
    /// (0, 0, 0) to (0, 0, cylinder_height), and
    /// the cone is from (0, 0, cylinder_height)
    /// to (0, 0, cylinder_height + cone_height).
    /// The cone will be split into \param resolution segments.
    /// The \param cylinder_height will be split into \param cylinder_split
    /// segments. The \param cone_height will be split into \param cone_split
    /// segments.
    static std::shared_ptr<TriangleMesh> CreateArrow(
            double cylinder_radius = 1.0,
            double cone_radius = 1.5,
            double cylinder_height = 5.0,
            double cone_height = 4.0,
            int resolution = 20,
            int cylinder_split = 4,
            int cone_split = 1);

    /// Factory function to create a coordinate frame mesh
    /// (TriangleMeshFactory.cpp) The coordinate frame will be centered at
    /// \param origin The x, y, z axis will be rendered as red, green, and blue
    /// arrows respectively. \param size is the length of the axes.
    static std::shared_ptr<TriangleMesh> CreateCoordinateFrame(
            double size = 1.0,
            const Eigen::Vector3d &origin = Eigen::Vector3d(0.0, 0.0, 0.0));

    /// Factory function to create a Moebius strip. \param length_split
    /// defines the number of segments along the Moebius strip, \param
    /// width_split defines the number of segments along the width of
    /// the Moebius strip, \param twists defines the number of twists of the
    /// strip, \param radius defines the radius of the Moebius strip,
    /// \param flatness controls the height of the strip, \param width
    /// controls the width of the Moebius strip and \param scale is used
    /// to scale the entire Moebius strip.
    static std::shared_ptr<TriangleMesh> CreateMoebius(int length_split = 70,
                                                       int width_split = 15,
                                                       int twists = 1,
                                                       double radius = 1,
                                                       double flatness = 1,
                                                       double width = 1,
                                                       double scale = 1);

protected:
    // Forward child class type to avoid indirect nonvirtual base
    TriangleMesh(Geometry::GeometryType type) : MeshBase(type) {}

    void FilterSmoothLaplacianHelper(
            std::shared_ptr<TriangleMesh> &mesh,
            const std::vector<Eigen::Vector3d> &prev_vertices,
            const std::vector<Eigen::Vector3d> &prev_vertex_normals,
            const std::vector<Eigen::Vector3d> &prev_vertex_colors,
            const std::vector<std::unordered_set<int>> &adjacency_list,
            double lambda,
            bool filter_vertex,
            bool filter_normal,
            bool filter_color) const;

public:
    std::vector<Eigen::Vector3i> triangles_;
    std::vector<Eigen::Vector3d> triangle_normals_;
    std::vector<std::unordered_set<int>> adjacency_list_;
    std::vector<Eigen::Vector2d> triangle_uvs_;
};

}  // namespace geometry
}  // namespace open3d
