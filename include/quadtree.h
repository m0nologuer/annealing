#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <algorithm>

using namespace std; 

#define max_triangles_per_leaf 5


class Triangle
{
  Eigen::Vector3d normal;
public:
  Eigen::Vector3d points[3];
  Triangle();
  Triangle(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c);
  Triangle( const Triangle& other );
  Eigen::Vector3d shortestDistanceTo(Triangle* other);
  Eigen::Vector3d shortestDistanceTo(Eigen::Vector3d point);
};

class QuadtreeNode
{
  bool leaf;

  QuadtreeNode* next_level[8];
  std::vector<Triangle*> triangles;
  int triangle_count;

  Eigen::Vector3d bounding_box_max;
  Eigen::Vector3d bounding_box_min;

public:
  QuadtreeNode();
  QuadtreeNode(std::vector<Triangle*> vertices, Eigen::Vector3d box_min, Eigen::Vector3d box_max);
  void updateShortestDistanceTo(QuadtreeNode * tree2, Eigen::Vector3d& min_vector);
  ~QuadtreeNode();

  static int comparison_index(Eigen::Vector3d vertex, Eigen::Vector3d medium);

};
