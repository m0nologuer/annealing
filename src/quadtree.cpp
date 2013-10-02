#include "quadtree.h"

int QuadtreeNode::comparison_index(Eigen::Vector3f vertex, Eigen::Vector3f medium)
{
  int node_index = 0;
      
      if (vertex(0) > medium(0))
        node_index += 4;
      if (vertex(1) > medium(1))
        node_index += 2;
      if (vertex(2) > medium(2))
        node_index += 1;
    return node_index;
}
void QuadtreeNode::updateShortestDistanceTo(QuadtreeNode * tree2, Eigen::Vector3f& min_vector){

    if (leaf)
    {
      if (tree2->leaf)
      {
        float min_distance = min_vector.norm();
        for (int i = 0; i < vertex_points; ++i)
          for (int j = 0; j < tree2->vertex_points; ++j)
          {
            Eigen::Vector3f difference = points[i]-tree2->points[j];
            if (difference.norm() < min_distance)
            {
              min_distance = difference.norm();
              min_vector = difference;
            }
          }
      }
      else
      {
        for (int i = 0; i < 8; ++i)
          updateShortestDistanceTo(tree2->next_level[i], min_vector);
      }
    }
    else
    {
      if (tree2->leaf)
      {
        for (int i = 0; i < 8; ++i)
          next_level[i]->updateShortestDistanceTo(tree2, min_vector);
      }
      else
      {
        for (int i = 0; i < 8; ++i)
          next_level[i]->updateShortestDistanceTo(tree2->next_level[i], min_vector);
      }

    }
  }

  QuadtreeNode::~QuadtreeNode(){
    for (int i = 0; i < 8; ++i)
      if (next_level[i])
        delete[] next_level[i];
  }

  QuadtreeNode::QuadtreeNode(std::vector<Eigen::Vector3f> vertices, Eigen::Vector3f box_min, Eigen::Vector3f box_max){
    bounding_box_min = box_min;
    bounding_box_max = box_max;

    if (!(vertices.size() > max_vertices_per_leaf ))
    { 
      leaf = true;
      for (int i = 0; i < 8; ++i)
        next_level[i] = NULL;
      vertex_points = vertices.size();
      for (int i = 0; i < vertices.size(); ++i)
        points[i] = vertices[i];
    }
    else
    {
      leaf = false;
      std::vector<Eigen::Vector3f> vector_lists[8];
      Eigen::Vector3f medium = (bounding_box_min+bounding_box_max)*0.5;
      for (int i = 0; i < vertices.size(); ++i)
      {
        int node_index = comparison_index(vertices[i], medium);
        vector_lists[node_index].push_back(vertices[i]);
      } 
      for (int i = 0; i < 8; ++i)
      {
        Eigen::Vector3f min, max;

        if ((i/4) == 0) {min(0) = box_min(0); max(0) = medium(0);}
        else {min(0) = medium(0); max(0) = box_max(0);}

        if (((i%4)/2) == 0) {min(1) = box_min(1); max(1) = medium(1);}
        else {min(1) = medium(1); max(1) = box_max(1);}

        if ((i%2) == 2) {min(2) = box_min(2); max(2) = medium(2);}
        else {min(2) = medium(2); max(2) = box_max(2);}

        next_level[i] = new QuadtreeNode(vector_lists[i],min,max);
      }
    }
  }
