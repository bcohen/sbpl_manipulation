/** /author Andrew Dornbush */

#ifndef _SBPL_GEOMETRY_UTILS_
#define _SBPL_GEOMETRY_UTILS_

#include <vector>
#include <math.h>
#include <cstdlib>
#include <cassert>
#include <Eigen/Core>
#include <geometry_msgs/Point.h>

namespace sbpl_geometry_utils
{

struct Point
{
	Point() { x = 0.0; y = 0.0; z = 0.0; }
	Point(double x, double y, double z) { this->x = x; this->y = y; this->z = z; }

  double x;
  double y;
  double z;
};

struct Triangle
{
	Point p1;
	Point p2;
	Point p3;
};

struct Sphere
{
  Point p;
  double radius;
};

bool getAxisAlignedBoundingBox(const std::vector<geometry_msgs::Point>& vertices, 
                               double& minX, double& minY, double& minZ, double& maxX, double& maxY, double& maxZ);
                        
double dot(const Eigen::Vector3d& a, const Eigen::Vector3d& b);  

Eigen::Vector3d cross(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
        
bool pointOnTriangle(const Eigen::Vector3d& point, const Eigen::Vector3d& vertex1, const Eigen::Vector3d& vertex2, 
                     const Eigen::Vector3d& vertex3);

bool intersects(const Triangle tr1, const Triangle& tr2, const double eps = 1.0e-4);

void createCubeMesh(double x, double y, double z, double length, std::vector<Triangle>& trianglesOut);

void getEnclosingSpheresOfCube(double xSize, double ySize, double zSize, double radius, std::vector<Sphere>& spheres);

void getEnclosingSpheresOfCube(double xSize, double ySize, double zSize, double radius, std::vector<std::vector<double> >& spheres);

////////////////////////////////////////////////////////////////////////////////
/// getEnclosingSpheresOfMesh
///
/// Generates a set of spheres that completely enclose a triangle mesh
///
/// @param vertices The list of vertices
/// @param triangles The list of indices into the vertex array (Triangle k is
///                  represented by by vertices[3*k + 0], vertices[3*k + 1] and
///                  vertices[3*k + 2]
/// @param radius The desired radius of the generated spheres
/// @param spheres The returned spheres
/// @param fillMesh A flag denoting whether there should also be spheres that
///                 fill in the mesh interior; false denotes only spheres
///                 enclosing the surface of the mesh are generated
////////////////////////////////////////////////////////////////////////////////
void getEnclosingSpheresOfMesh(const std::vector<geometry_msgs::Point>& vertices,
                               const std::vector<int>& triangles,
                               double radius, std::vector<Sphere>& spheres,
                               bool fillMesh = false);
                               
}
#endif
