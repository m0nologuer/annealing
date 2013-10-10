#include "quadtree.h"

class Mesh
{
  Eigen::Vector3d* vertexBuffer;
  int* indexBuffer;

  int vertex_count;
  int face_count;

  Eigen::Vector3d current_closest_point;
  Eigen::Vector3d prev_closest_point;

  Eigen::Vector3d current_center;
  Eigen::Vector3d prev_center;
  double boundingSphere;

public:

  Mesh();
  ~Mesh();

  void moveToOrigin();
  void update();
  void boundingBox(double &x, double &x_max, double &y, double &y_max, double &z, double &z_max);
  void boundingBoxSize(double &x, double &y, double &z);
  void translate(Eigen::Vector3d translation);
  void rotate(Eigen::Matrix3d rotation, Eigen::Vector3d about);
  void move(Eigen::Vector3d translation);

  void buildQuadtree(QuadtreeNode** out_tree, double cube_size);
  void updateMinDistance(Mesh* secondMesh, double cube_size, double& distance, Eigen::Vector3d& vector_to_closest_object);
  void rotateLessThan(double max_rotation, Eigen::Vector3d& vector_to_closest_object);
  Eigen::Vector3d smallestVectorToCube(double cube_size);

  void write(char* out_obj_file);
  static void meshFromFile(char* filename, Mesh* out_mesh);
  static void concatenate(Mesh* meshArray, int mesh_count, Mesh* out_mesh);
};
