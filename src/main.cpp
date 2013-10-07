// reading a text file
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include "mesh.h"

#define GAP 0.1f
#define PADDING 30.0f
#define PERCENT_TRANSLATION 0.5
#define PERCENT_ROTATION 0.5
#define ITERATIONS 100
#define SPACING 5.0
#define CUBE_SHRINKAGE_RATE 0.01
#define CONST_PI 3.14

using namespace std;

int main (int argc, char *argv[]) {
  
  int meshCount = argc-1;

  Mesh * meshes = new Mesh[meshCount];
  for (int i = 1; i < argc; ++i)
    Mesh::meshFromFile(argv[i], &meshes[i-1]);

  //intital placement,start by stacking them on top of each other at hapharzrd rotations
  float x_coord, y_coord, z_coord = GAP;
  z_coord += PADDING;

  for (int i = 0; i < meshCount; ++i)
  {
    Eigen::Vector3f random_direction((rand()%100)*0.01,(rand()%100)*0.01,(rand()%100)*0.01);
    random_direction.normalize();
    meshes[i].rotate(Eigen::AngleAxisf((rand()%100)*0.01*CONST_PI, random_direction)*Eigen::Scaling(1.0f), Eigen::Vector3f(0,0,0));
    
    meshes[i].moveToOrigin();
    float x,y,z;
    meshes[i].boundingBoxSize(x,y,z);
    meshes[i].move(Eigen::Vector3f(GAP+ PADDING,GAP+PADDING,z_coord));

    x_coord = max(x, x_coord);
    y_coord = max(y, x_coord);
    z_coord += (z+GAP+SPACING);
  }

  Mesh finalMesh;
  Mesh::concatenate(meshes, meshCount, &finalMesh);
  finalMesh.write("output_start.obj");

  float cube_size = max(x_coord+2.0f*(PADDING+GAP), max(y_coord+2.0f*(PADDING+GAP), z_coord+PADDING*3));
  int counter = 0;

  do
  {
    for (int i = 0; i < meshCount; ++i)
    {
      //find closest distance
      Eigen::Vector3f vector_to_closest_object = meshes[i].smallestVectorToCube(cube_size);
      float closest_distance = vector_to_closest_object.norm();

      for (int j = 0; j < meshCount; ++j)
        if (i != j)
          meshes[i].updateMinDistance(&meshes[j], cube_size, closest_distance, vector_to_closest_object);

      //rotate and translate
      float translation_distance = (closest_distance- GAP)*PERCENT_TRANSLATION;
      float rotation_distance = (closest_distance- GAP)*PERCENT_ROTATION;

      meshes[i].move(vector_to_closest_object*translation_distance/closest_distance);
      meshes[i].rotateLessThan(rotation_distance,vector_to_closest_object);

    }
    
    //adjust cube
    
    float cube_max = 0; 
    float cube_min = cube_size;
    for (int i = 0; i < meshCount; ++i)
    {
      float x,y,z,a,b,c;
      meshes[i].boundingBox(a,x,b,y,c,z);
      cube_max = max(x,max(y,max(z,cube_max)));
      cube_min = max(a,max(b,max(c,cube_min)));
    }
    float real_cube_size = (cube_min+cube_max)*0.5 + GAP*4;
    cube_size = real_cube_size*CUBE_SHRINKAGE_RATE + cube_size*(1- CUBE_SHRINKAGE_RATE);
    float start = -(real_cube_size- cube_size)*0.5;
    for (int i = 0; i < meshCount; ++i)
      meshes[i].move(Eigen::Vector3f(start, start, start));


    counter++;
  } while (counter < ITERATIONS);

  Mesh::concatenate(meshes, meshCount, &finalMesh);
  finalMesh.write("output_file.obj");


  delete[] meshes;

  return 0;
} 