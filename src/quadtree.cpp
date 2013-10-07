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
        for (int i = 0; i < triangle_count; ++i)
          for (int j = 0; j < tree2->triangle_count; ++j)
          {
            Eigen::Vector3f difference = triangles[i].shortestDistanceTo(tree2->triangles[j]);
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
 //  for (int i = 0; i < 8; ++i)
  //    if (next_level[i])
   //     delete[] next_level[i];
  }

  QuadtreeNode::QuadtreeNode(std::vector<Triangle*> vertices, Eigen::Vector3f box_min, Eigen::Vector3f box_max){
    bounding_box_min = box_min;
    bounding_box_max = box_max;

    if (!(vertices.size() > max_triangles_per_leaf ))
    { 
      leaf = true;
      for (int i = 0; i < 8; ++i)
        next_level[i] = NULL;
      triangle_count = vertices.size();
      for (int i = 0; i < vertices.size(); ++i)
        triangles[i] = *vertices[i];
    }
    else
    {
      leaf = false;
      std::vector<Triangle*> vector_lists[8];
      Eigen::Vector3f medium = (bounding_box_min+bounding_box_max)*0.5;
      for (int i = 0; i < vertices.size(); ++i)
      {
        for (int j = 0; j < 3; j++)
        {
          int node_index = comparison_index(vertices[i]->points[j], medium);
          if (!(std::find(vector_lists[node_index].begin(), vector_lists[node_index].end(), vertices[i])
           != vector_lists[node_index].end())) 
            vector_lists[node_index].push_back(vertices[i]);
        }
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

Triangle::Triangle(){};
Triangle::Triangle(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c){
  points[0] = a;
  points[1] = b;
  points[2] = c;

  normal = ((b-a).cross(c-b)).normalized();
};
Triangle::Triangle( const Triangle& other ){
  for (int i = 0; i < 3; ++i)
    points[i] = other.points[i];
}
Eigen::Vector3f Triangle::shortestDistanceTo(Eigen::Vector3f point){

  Eigen::Vector3f projected_point = point - points[0];
  float bary1 = projected_point.dot(points[1]-points[0]);

  projected_point = projected_point - bary1*(points[1]-points[0]);
  float bary2 = projected_point.dot(points[2]-points[0]);

  if (bary1 >= 0 && bary1 <= 1 && bary2 >= 0 && bary2 <= 1)
  {
    return ((bary1*(points[1]-points[0])-bary2*(points[2]-points[0]))+points[0] - point);
  }
  else
  {
    Eigen::Vector3f distance = points[0] - point;
    for (int i = 1; i < 3; ++i)
    {
      Eigen::Vector3f new_dist = points[i] - point;
      if (new_dist.norm() < distance.norm())
        distance = new_dist;
    }
    return distance;
  }
}
Eigen::Vector3f Triangle::shortestDistanceTo(Triangle other){

  Eigen::Vector3f distance = shortestDistanceTo(other.points[0]);
  for (int i = 1; i < 3; ++i)
  {
    Eigen::Vector3f new_dist = shortestDistanceTo(other.points[i]);
    if (new_dist.norm() < distance.norm())
      distance = new_dist;
  }

  for (int i = 1; i < 3; ++i)
  {
    Eigen::Vector3f new_dist = other.shortestDistanceTo(points[i]);
    if (new_dist.norm() < distance.norm())
      distance = new_dist;
  }
  return distance;
}
