// reading a text file
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include "mesh.h"

#define GAP 3.0
#define PADDING 50.0f
#define PERCENT_TRANSLATION 0.25f
#define PERCENT_ROTATION 0.35f
#define ITERATIONS 400
#define SPACING 20
#define CUBE_SHRINKAGE_RATE 0.01
#define CONST_PI 3.14

using namespace std;

int main (int argc, char *argv[]) {
  
  int meshCount = argc-1;

  Mesh * meshes = new Mesh[meshCount];
  for (int i = 1; i < argc; ++i)
    Mesh::meshFromFile(argv[i], &meshes[i-1]);

  //intital placement,start by stacking them on top of each other at hapharzrd rotations
  double x_coord, y_coord, z_coord = GAP;
  z_coord += PADDING;

  for (int i = 0; i < meshCount; ++i)
  {
    Eigen::Vector3d random_direction((rand()%100)*0.01,(rand()%100)*0.01,(rand()%100)*0.01);
    random_direction.normalize();
    meshes[i].rotate(Eigen::AngleAxisd((rand()%100)*0.01*CONST_PI, random_direction)*Eigen::Scaling(1.0), Eigen::Vector3d(0,0,0));
    
    meshes[i].moveToOrigin();
    double x,y,z;
    meshes[i].boundingBoxSize(x,y,z);
    meshes[i].move(Eigen::Vector3d(GAP+ PADDING,GAP+PADDING,z_coord));

    x_coord = max(x, x_coord);
    y_coord = max(y, x_coord);
    z_coord += (z+GAP+SPACING);
  }

  Mesh finalMesh;
  Mesh::concatenate(meshes, meshCount, &finalMesh);
  finalMesh.write("output_start.obj");

  double cube_size = max(x_coord+2.0f*(PADDING+GAP), max(y_coord+2.0f*(PADDING+GAP), z_coord+PADDING*3));
  int counter = 0;

  do
  {
    for (int i = 0; i < meshCount; ++i)
    {
      //find closest distance
      Eigen::Vector3d vector_to_closest_object = meshes[i].smallestVectorToCube(cube_size);
      double closest_distance = vector_to_closest_object.norm();

      for (int j = 0; j < meshCount; ++j)
        if (i != j)
          meshes[i].updateMinDistance(&meshes[j], cube_size, closest_distance, vector_to_closest_object);

      //rotate and translate
      double translation_distance = (max(closest_distance,GAP)- GAP)*PERCENT_TRANSLATION;
      double rotation_distance = (max(closest_distance,GAP)- GAP)*PERCENT_ROTATION;

      meshes[i].rotateLessThan(rotation_distance,vector_to_closest_object);
      meshes[i].move(vector_to_closest_object*translation_distance/closest_distance);
    }
    
    //adjust cube
    
    double cube_max = 0; 
    double cube_min = cube_size;
    for (int i = 0; i < meshCount; ++i)
    {
      double x,y,z,a,b,c;
      meshes[i].boundingBox(a,x,b,y,c,z);
      cube_max = max(x,max(y,max(z,cube_max)));
      cube_min = max(a,max(b,max(c,cube_min)));
    }
    double real_cube_size = (cube_min+cube_max)*0.5 + GAP*4;
    cube_size = real_cube_size*CUBE_SHRINKAGE_RATE + cube_size*(1- CUBE_SHRINKAGE_RATE);
    double start = -(real_cube_size- cube_size)*0.5;
    for (int i = 0; i < meshCount; ++i)
      meshes[i].translate(Eigen::Vector3d(start, start, start));


    counter++;
  } while (counter < ITERATIONS);

  Mesh::concatenate(meshes, meshCount, &finalMesh);
  finalMesh.write("output_file.obj");


  delete[] meshes;

  return 0;
} 