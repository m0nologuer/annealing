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

#define max_triangles_per_leaf 10


class Triangle
{
  Eigen::Vector3f normal;
public:
  Eigen::Vector3f points[3];
  Triangle();
  Triangle(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c);
  Triangle( const Triangle& other );
  Eigen::Vector3f shortestDistanceTo(Triangle other);
  Eigen::Vector3f shortestDistanceTo(Eigen::Vector3f point);
};

class QuadtreeNode
{
  bool leaf;

  QuadtreeNode* next_level[8];
  Triangle triangles[max_triangles_per_leaf];
  int triangle_count;

  Eigen::Vector3f bounding_box_max;
  Eigen::Vector3f bounding_box_min;

public:
  QuadtreeNode();
  QuadtreeNode(std::vector<Triangle*> vertices, Eigen::Vector3f box_min, Eigen::Vector3f box_max);
  void updateShortestDistanceTo(QuadtreeNode * tree2, Eigen::Vector3f& min_vector);
  ~QuadtreeNode();

  static int comparison_index(Eigen::Vector3f vertex, Eigen::Vector3f medium);

};
