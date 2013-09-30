#include "quadtree.h"

class Mesh
{
  Eigen::Vector3f* vertexBuffer;
  int* indexBuffer;
  QuadtreeNode* quad_tree_start;

  int vertex_count;
  int face_count;

  Eigen::Vector3f current_center;
  Eigen::Vector3f prev_center;
  float boundingSphere;

public:

  Mesh();
  ~Mesh();

  void moveToOrigin();
  void boundingBox(float &x, float &x_max, float &y, float &y_max, float &z, float &z_max);
  void boundingBoxSize(float &x, float &y, float &z);

  void rotate(Eigen::Matrix3f rotation, Eigen::Vector3f about);
  void move(Eigen::Vector3f translation);

  void updateMinDistance(Mesh* secondMesh, float& distance, Eigen::Vector3f& vector_to_closest_object);
  void rotateLessThan(float max_rotation, Eigen::Vector3f& vector_to_closest_object);
  Eigen::Vector3f smallestVectorToCube(float cube_size);

  void write(char* out_obj_file);
  static void meshFromFile(char* filename, Mesh* out_mesh);
};
