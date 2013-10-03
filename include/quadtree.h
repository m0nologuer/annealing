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

#define max_vertices_per_leaf 5

class QuadtreeNode
{
  bool leaf;

  QuadtreeNode* next_level[8];
  Eigen::Vector3f points[max_vertices_per_leaf];
  int vertex_points;

  Eigen::Vector3f bounding_box_max;
  Eigen::Vector3f bounding_box_min;

public:
  QuadtreeNode();
  QuadtreeNode(std::vector<Eigen::Vector3f> vertices, Eigen::Vector3f box_min, Eigen::Vector3f box_max);
  void updateShortestDistanceTo(QuadtreeNode * tree2, Eigen::Vector3f& min_vector);
  ~QuadtreeNode();

  static int comparison_index(Eigen::Vector3f vertex, Eigen::Vector3f medium);

};
