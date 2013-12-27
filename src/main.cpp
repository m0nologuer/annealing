// reading a text file
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include "mesh.h"

#define GAP 1.0
#define PADDING 5.0f
#define PERCENT_TRANSLATION 0.2f
#define PERCENT_ROTATION 0.99f
#define ITERATIONS 10000
#define SPACING 5
#define CUBE_SHRINKAGE_RATE 0.1
#define CONST_PI 3.14

using namespace std;

int main (int argc, char *argv[]) {

  int meshCount = argc-1;

  Mesh * meshes = new Mesh[meshCount];
  for (int i = 1; i < argc; ++i)
    Mesh::meshFromFile(argv[i], &meshes[i-1]);

  //intital placement,start by stacking them on top of each other at hapharzrd rotations
  double x_coord, y_coord, z_coord = SPACING*2;
  for (int i = 0; i < meshCount; ++i)
  {
    Eigen::Vector3d random_vector((rand()%100)*0.01,(rand()%100)*0.01,(rand()%100)*0.01);
    random_vector.normalize();
    double random_angle = (rand()%100)*0.0628;
    meshes[i].rotate(Eigen::AngleAxisd(-random_angle, random_vector)*Eigen::Scaling(1.0), Eigen::Vector3d(0,0,0));
    meshes[i].moveToOrigin();
    double x,y,z;
    meshes[i].boundingBoxSize(x,y,z);

    x_coord = max(x + SPACING*2, x_coord);
    y_coord = max(y + SPACING*2, y_coord);
    z_coord = max(z + SPACING*2, z_coord);
  }
  int cube_count = max((int)cbrtf((float)meshCount)+1,1);
  for (int i = 0; i < meshCount; ++i){
    int x = i%cube_count;
    int y = (i/cube_count)%cube_count;
    int z = (i/(cube_count*cube_count))%cube_count;
    meshes[i].translate(Eigen::Vector3d(x*x_coord + PADDING, y*y_coord + PADDING ,z*z_coord + PADDING));
  }

  Mesh finalMesh;
  Mesh::concatenate(meshes, meshCount, &finalMesh);
  finalMesh.write("output_start.obj");

  double cube_size = max(x_coord*cube_count+2.0f*(PADDING+GAP*2), max(y_coord*cube_count+2.0f*(PADDING+GAP*2), z_coord*cube_count+PADDING*3));
  int counter = 0;
  bool still_moving = false;
  double pseudo_GAP = SPACING;

  do
  {
    still_moving = false;

    for (int i = 0; i < meshCount; ++i)
      for(int k = 0; k < 5; k++)
    {
      meshes[i].update();
      
      //find closest distance
      Eigen::Vector3d vector_to_closest_object = meshes[i].smallestVectorToCube(cube_size);
      double closest_distance = vector_to_closest_object.norm();

      for (int j = 0; j < meshCount; ++j)
        if (i != j)
          meshes[i].updateMinDistance(&meshes[j], cube_size, closest_distance, vector_to_closest_object);


      closest_distance = vector_to_closest_object.norm();

      //rotate and translate
      double translation_distance = (max(closest_distance,GAP)- GAP)*PERCENT_TRANSLATION;
      double rotation_distance = (max(closest_distance,GAP)- GAP)*PERCENT_ROTATION;

      cout << i << " " << closest_distance <<  " trans:" << translation_distance << " rotat:" << rotation_distance << endl;


      assert(closest_distance > GAP);


      if (closest_distance > GAP)
      {
        still_moving = true;
        
        //move it
        if (k >2)
          meshes[i].rotateLessThan(rotation_distance,vector_to_closest_object);
        else
          meshes[i].move(-vector_to_closest_object.normalized()*translation_distance);
      }      

    }
    
    //adjust cube
    
    double cube_max = 0; 
    double cube_min = cube_size;
    for (int i = 0; i < meshCount; ++i)
    {
      double x,y,z,a,b,c;
      meshes[i].boundingBox(a,x,b,y,c,z);
      cube_max = max(x,max(y,max(z,cube_max)));
      cube_min = min(a,min(b,min(c,cube_min)));
    }
    double real_cube_size = cube_max-cube_min + GAP*2;
    cube_size = real_cube_size *CUBE_SHRINKAGE_RATE + cube_size*(1- CUBE_SHRINKAGE_RATE);
    assert(real_cube_size < cube_size);
    double start = (cube_size-real_cube_size)*0.5 // to center the cube
                      -cube_min
                      + GAP; // or just above zero
    for (int i = 0; i < meshCount; ++i)
      meshes[i].translate(Eigen::Vector3d(start, start, start));


   cout <<"cube_size: " << cube_size << " occupied cube size: " << real_cube_size << endl;

    Mesh m;
    Mesh::concatenate(meshes, meshCount, &m);
    m.write("output_file.obj");

    counter++;
  } while (counter < ITERATIONS && still_moving);

  Mesh::concatenate(meshes, meshCount, &finalMesh);
  finalMesh.write("output_file.obj");


  delete[] meshes;

  return 0;
} 