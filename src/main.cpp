// reading a text file
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include "mesh.h"

#define GAP 0.1f
#define PERCENT_TRANSLATION 0.5
#define PERCENT_ROTATION 0.5
#define ITERATIONS 200
#define SPACING 0.8
#define CUBE_SHRINKAGE_RATE 1
#define CONST_PI 3.14

using namespace std;

int main (int argc, char *argv[]) {
  
  int meshCount = argc-1;

  Mesh * meshes = new Mesh[meshCount];
  for (int i = 1; i < argc; ++i)
    Mesh::meshFromFile(argv[i], &meshes[i-1]);

  //intital placement,start by stacking them on top of each other at hapharzrd rotations
  float x_coord, y_coord, z_coord = GAP*2;
  for (int i = 0; i < meshCount; ++i)
  {
    Eigen::Vector3f random_direction((rand()%100)*0.01,(rand()%100)*0.01,(rand()%100)*0.01);
    random_direction.normalize();
    meshes[i].rotate(Eigen::AngleAxisf((rand()%100)*0.01*CONST_PI, random_direction)*Eigen::Scaling(1.0f), Eigen::Vector3f(0,0,0));
    
    meshes[i].moveToOrigin();
    float x,y,z;
    meshes[i].boundingBoxSize(x,y,z);
    meshes[i].move(Eigen::Vector3f(GAP,GAP,z_coord));

    x_coord = max(x, x_coord);
    y_coord = max(y, x_coord);
    z_coord += (z+GAP);
  }

  float cube_size = max(x_coord+2.0f*GAP, max(y_coord+2.0f*GAP, z_coord));
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
          meshes[i].updateMinDistance(&meshes[j], closest_distance, vector_to_closest_object);

      //rotate and translate
      float translation_distance = (closest_distance- GAP)*PERCENT_TRANSLATION;
      float rotation_distance = (closest_distance- GAP)*PERCENT_ROTATION;

      meshes[i].move(vector_to_closest_object*translation_distance/closest_distance);
      meshes[i].rotateLessThan(rotation_distance,vector_to_closest_object);
    }
    
    cout << counter << endl;
    for (int i = 0; i < meshCount; ++i)
      meshes[i].write(argv[i+1]);
    cube_size *= CUBE_SHRINKAGE_RATE;
    counter++;
  } while (counter < ITERATIONS);

  for (int i = 0; i < meshCount; ++i)
    meshes[i].write(argv[i+1]);

  delete[] meshes;

  return 0;
} 