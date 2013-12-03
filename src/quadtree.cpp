#include "quadtree.h"
#define EPISILON 1e-14

int QuadtreeNode::comparison_index(Eigen::Vector3d vertex, Eigen::Vector3d medium)
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
double interval_distance(double x_start, double x_end, double y_start, double y_end)
{
  if (x_start > y_start)
  {
    // in the interval
    if (y_end > x_start)
      return 0;
    else
      return x_start - y_end;
  }
  else
  {
    if (x_end > y_start)
      return 0;
    else
      return y_start- x_end;

  }
}
double boundingBoxDistance(Eigen::Vector3d min1, Eigen::Vector3d min2, Eigen::Vector3d max1, Eigen::Vector3d max2)
{
  double x = interval_distance(min1(0), max1(0), min2(0), max2(0));
  double y = interval_distance(min1(1), max1(1), min2(1), max2(1));
  double z = interval_distance(min1(2), max1(2), min2(2), max2(2));

  return (Eigen::Vector3d(x,y,z)).norm();
}
void QuadtreeNode::updateShortestDistanceTo(QuadtreeNode * tree2, Eigen::Vector3d& min_vector, Eigen::Vector3d& closest_point){
  
  if (boundingBoxDistance(bounding_box_min, tree2->bounding_box_min, bounding_box_max, tree2->bounding_box_max) > min_vector.norm())
    return;

    if (leaf)
    {
      if (tree2->leaf)
      {
        double min_distance = min_vector.norm();
        for (int i = 0; i < triangle_count; ++i)
          for (int j = 0; j < tree2->triangle_count; ++j)
          {
            Eigen::Vector3d close_point;
            Eigen::Vector3d difference = triangles[i]->shortestDistanceTo(tree2->triangles[j], close_point);
            if (difference.norm() < min_distance)
            {
              min_distance = difference.norm();
              min_vector = difference;
              closest_point = close_point;
            }
          }
      }
      else
      {
        for (int i = 0; i < 8; ++i)
          updateShortestDistanceTo(tree2->next_level[i], min_vector, closest_point);
      }
    }
    else
    {
      if (tree2->leaf)
      {
        for (int i = 0; i < 8; ++i)
          next_level[i]->updateShortestDistanceTo(tree2, min_vector, closest_point);
      }
      else
      {
        for (int i = 0; i < 8; ++i)
          next_level[i]->updateShortestDistanceTo(tree2->next_level[i], min_vector, closest_point);
      }

    }
  }

  QuadtreeNode::~QuadtreeNode(){
 //  for (int i = 0; i < 8; ++i)
  //    if (next_level[i])
   //     delete[] next_level[i];
  }

  QuadtreeNode::QuadtreeNode(std::vector<Triangle*> vertices, Eigen::Vector3d box_min, Eigen::Vector3d box_max){
    bounding_box_min = box_min;
    bounding_box_max = box_max;
    Eigen::Vector3d medium = (bounding_box_min+bounding_box_max)*0.5;

    bool can_subdivide = true;

    for (int i = 0; i < vertices.size(); ++i)
    {
      int node_index = comparison_index(vertices[i]->points[0], medium);
      for (int j = 1; j < 3; j++)
        if (comparison_index(vertices[i]->points[j], medium) != node_index)
        {
          can_subdivide = false;
          break;
        }
    }

    if (!(vertices.size() > max_triangles_per_leaf ) || !can_subdivide)
    { 
      leaf = true;
      for (int i = 0; i < 8; ++i)
        next_level[i] = NULL;
      triangle_count = vertices.size();
      for (int i = 0; i < vertices.size(); ++i)
        triangles.push_back(vertices[i]);
    }
    else
    {
      leaf = false;
      std::vector<Triangle*> vector_lists[8];
      for (int i = 0; i < vertices.size(); ++i)
      {
        int node_index = comparison_index(vertices[i]->points[0], medium);
        if (!(std::find(vector_lists[node_index].begin(), vector_lists[node_index].end(), vertices[i])
           != vector_lists[node_index].end())) 
            vector_lists[node_index].push_back(vertices[i]);
        
      } 
      for (int i = 0; i < 8; ++i)
      {
        Eigen::Vector3d min, max;

        if (i < 4) {min(0) = box_min(0); max(0) = medium(0);}
        else {min(0) = medium(0); max(0) = box_max(0);}

        if ((i%4) < 2) {min(1) = box_min(1); max(1) = medium(1);}
        else {min(1) = medium(1); max(1) = box_max(1);}

        if ((i%2) < 1) {min(2) = box_min(2); max(2) = medium(2);}
        else {min(2) = medium(2); max(2) = box_max(2);}

        next_level[i] = new QuadtreeNode(vector_lists[i],min,max);
      }
    }
  }
Triangle::Triangle(){};
Triangle::Triangle(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c){
  points[0] = a;
  points[1] = b;
  points[2] = c;

  normal = ((b-a).cross(c-b)).normalized();
};
Triangle::Triangle( const Triangle& other ){
  for (int i = 0; i < 3; ++i)
    points[i] = other.points[i];
}
//line triangle intersections
Eigen::Vector3d Triangle::shortestDistanceTo(Eigen::Vector3d line_segment_start, Eigen::Vector3d line_segment_end, Eigen::Vector3d& mesh_closest_point)
{
  Eigen::Vector3d shortest_dist(BIG_DOUBLE,BIG_DOUBLE,BIG_DOUBLE);

  //line intersection with all of the edges
  Eigen::Vector3d normal = ((points[1]-points[0]).cross(points[2]-points[1])).normalized();
  float d = points[1].dot(normal);
  Eigen::Vector3d p = line_segment_start - (line_segment_start.dot(normal) -d)*normal;
  Eigen::Vector3d r = (line_segment_end - (line_segment_end.dot(normal)-d)*normal) - p;

  for (int i = 0; i < 3; ++i)
  {
    Eigen::Vector3d q = points[i];
    Eigen::Vector3d s = points[(i+1)%3] - q;

    double u = ((q-p).cross(r)).norm()/(r.cross(s)).norm();
    double t = ((q-p).cross(s)).norm()/(r.cross(s)).norm();

    if (u > -EPISILON && u < 1+ EPISILON && t > -EPISILON && t < 1+EPISILON)
    {
      Eigen::Vector3d closest_point = q + u*s;
      Eigen::Vector3d mesh_close_point = (line_segment_start*(1-t) + line_segment_end*t);
      Eigen::Vector3d short_distance = mesh_close_point - closest_point;

      if (short_distance.norm() < shortest_dist.norm())
      {
        shortest_dist = short_distance;
        mesh_closest_point = mesh_close_point;
      }
    }

  }
  return shortest_dist;
}
void update_shortest_distance(Eigen::Vector3d new_dist, Eigen::Vector3d& old_dist){
  if (old_dist.norm() > new_dist.norm())
    old_dist = new_dist;
}
//point triangle intersections

Eigen::Vector3d Triangle::shortestDistanceTo(Eigen::Vector3d point){

   //first go through vertices and pick the cloests.
  Eigen::Vector3d distance = points[0] - point;
  for (int i = 1; i < 3; ++i)
  {
    Eigen::Vector3d new_dist = points[i] - point;
    update_shortest_distance(new_dist, distance);
  }

  //use barycentric coordinates to find shortest distance from a triangle to a point
  Eigen::Vector3d normal = ((points[1]-points[0]).cross(points[2]-points[1])).normalized();
  Eigen::Vector3d projected_point = point - point.dot(normal)*normal;

  Eigen::Vector3d v0 = points[2]-points[0];
  Eigen::Vector3d v1 = points[1]-points[0];
  Eigen::Vector3d v2 = projected_point-points[0];

  // Compute dot products
  double dot00 = v0.dot(v0);
  double dot01 = v0.dot(v1);
  double dot02 = v0.dot(v2);
  double dot11 = v1.dot(v1);
  double dot12 = v1.dot(v2);

// Compute barycentric coordinates
  double denom = (dot00 * dot11 - dot01 * dot01);
  double u = (dot11 * dot02 - dot01 * dot12) ;
  double v = (dot00 * dot12 - dot01 * dot02) ;

  //project point onto all three edges
  double edge1 = (point-points[0]).dot(points[1]-points[0]); 
  double edge1_denom = (points[1]-points[0]).squaredNorm();
  double edge2 = (point-points[1]).dot(points[2]-points[1]);
  double edge2_denom = (points[2]-points[1]).squaredNorm();
  double edge3 = (point-points[2]).dot(points[0]-points[2])/(points[0]-points[2]).squaredNorm();
  double edge3_denom = (points[2]-points[0]).squaredNorm();

  // Check if projected point is in triangle
  if  ((u > 0) && (v > 0) && (u + v < denom))
  {
    u = u/denom;
    v = v/denom;
    Eigen::Vector3d closest_point = points[0] + v * (points[1] - points[0]) + u * (points[2]-points[0]);

    update_shortest_distance((closest_point - point), distance);
  }
  if (edge1 > 0 && edge1 < edge1_denom)
  {
    edge1 = edge1/edge1_denom;
    Eigen::Vector3d closest_point = edge1*points[1] + (1- edge1)*points[0];
    update_shortest_distance((closest_point - point), distance);
  }
  if (edge2 > 0 && edge2 < edge2_denom)
  {
    edge2 = edge2/edge2_denom;
    Eigen::Vector3d closest_point = edge2*points[2] + (1- edge2)*points[1];
    update_shortest_distance((closest_point - point), distance);
  }
  if (edge3 > 0 && edge3 < edge3_denom)
  {
    edge3 = edge3/edge3_denom;
    Eigen::Vector3d closest_point = edge3*points[0] + (1- edge3)*points[2];
    update_shortest_distance((closest_point - point), distance);
  }
  
  return distance;
}
Eigen::Vector3d Triangle::shortestDistanceTo(Triangle* other, Eigen::Vector3d& closest_point){

  Eigen::Vector3d distance = shortestDistanceTo(other->points[0]);
  closest_point = distance + other->points[0];

  //project other triangle vertices onto us
  for (int i = 1; i < 3; ++i)
  {
    Eigen::Vector3d new_dist = shortestDistanceTo(other->points[i]);
    if (new_dist.norm() < distance.norm())
    {
      distance = new_dist;
      closest_point = distance + other->points[i];
    }
  }
  
  //project all vertices onto other triangle
  for (int i = 0; i < 3; ++i)
  {
    Eigen::Vector3d new_dist = -other->shortestDistanceTo(points[i]);
    if (new_dist.norm() < distance.norm())
    {
      distance = new_dist;
      closest_point = points[i];
    }
  }
  
  //project edges onto other triangle, do line intersections

  for (int i = 0; i < 3; ++i)
  {
    Eigen::Vector3d close_point;
    Eigen::Vector3d new_dist = -shortestDistanceTo(other->points[i], other->points[(i+1)%3], close_point);
    if (new_dist.norm() < distance.norm())
    {
      distance = new_dist;
      closest_point = close_point;
    }
  }

  for (int i = 0; i < 3; ++i)
  {
    Eigen::Vector3d close_point;
    Eigen::Vector3d new_dist = other->shortestDistanceTo(points[i],points[(i+1)%3], close_point);
    if (new_dist.norm() < distance.norm())
    {
      distance = new_dist;
      closest_point = points[i];
    }
  }

  return distance;
}
