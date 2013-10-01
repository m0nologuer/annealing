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
  Eigen::Vector3f QuadtreeNode::shortestDistanceTo(QuadtreeNode * tree2){

    if (leaf)
    {
      if (tree2->leaf)
      {
        float min_distance = (points[0] -tree2->points[0]).norm();
        Eigen::Vector3f min_vector = points[0] - tree2->points[0];
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
          return min_vector;
      }
      else
      {
        Eigen::Vector3f medium = (tree2->bounding_box_max+tree2->bounding_box_min)*0.5;
        Eigen::Vector3f medium1 = (bounding_box_max+bounding_box_min)*0.5;
        int node_index = comparison_index(medium, medium1);

        //find correct box & call again
        return next_level[node_index]->shortestDistanceTo(tree2);
      }
    }
    else
    {
      Eigen::Vector3f medium = (tree2->bounding_box_max+tree2->bounding_box_min)*0.5;
      Eigen::Vector3f medium1 = (bounding_box_max+bounding_box_min)*0.5;
      int node_index = comparison_index(medium1, medium);

      //find correct box & call again
      return shortestDistanceTo(tree2->next_level[node_index]);

    }
  }

  QuadtreeNode::~QuadtreeNode(){
    for (int i = 0; i < 8; ++i)
      if (next_level[i])
        delete[] next_level[i];
  }

  QuadtreeNode::QuadtreeNode(std::vector<Eigen::Vector3f> vertices){

    if (vertices.size() == 0)
    {
      bounding_box_max = Eigen::Vector3f(0,0,0);
      bounding_box_min = Eigen::Vector3f(0,0,0);
    }
    else
    {
      bounding_box_min = vertices[0];
      bounding_box_max = vertices[0];
    }
    for (int i = 0; i < vertices.size(); ++i)
    {
      if (vertices[i](0) < bounding_box_min(0))
        bounding_box_min(0) = vertices[i](0);
      if (vertices[i](0) > bounding_box_max(0))
        bounding_box_max(0) = vertices[i](0);
      if (vertices[i](1) < bounding_box_min(1))
        bounding_box_min(1) = vertices[i](1);
      if (vertices[i](1) > bounding_box_max(1))
        bounding_box_max(1) = vertices[i](1);
      if (vertices[i](2) < bounding_box_min(2))
        bounding_box_min(2) = vertices[i](2);
      if (vertices[i](2) > bounding_box_max(2))
        bounding_box_max(2) = vertices[i](2);
    }


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
        next_level[i] = new QuadtreeNode(vector_lists[i]);
    }
  }
