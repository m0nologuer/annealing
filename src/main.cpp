// reading a text file
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include "mesh.h"

#define GAP 1.0
#define PADDING 5.0f
#define PERCENT_TRANSLATION 0.4f
#define PERCENT_ROTATION 0.6f
#define ITERATIONS 10000
#define SPACING 5
#define CUBE_SHRINKAGE_RATE 0.01
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

  do
  {
    still_moving = false;

    for (int i = 0; i < meshCount; ++i)
      for (int turn = 0; turn < 6; turn++)
    {      
      //find closest distance
      Eigen::Vector3d vector_to_closest_object = meshes[i].smallestVectorToCube(cube_size);
      double closest_distance = vector_to_closest_object.norm();

      int closest_mesh = i;
      for (int j = 0; j < meshCount; ++j)
        if (i != j)
        {
          meshes[i].updateMinDistance(&meshes[j], cube_size, closest_distance, vector_to_closest_object);
          closest_mesh = j;
        }

      closest_distance = vector_to_closest_object.norm();

      double total_mass = meshes[i].getMass() + meshes[closest_mesh].getMass();

      //find closest distance from other object
      Eigen::Vector3d vector_to_closest_object2 = meshes[closest_mesh].smallestVectorToCube(cube_size);
      for (int j = 0; j < meshCount; ++j)
        if (closest_mesh != j)
          meshes[closest_mesh].updateMinDistance(&meshes[j], cube_size, closest_distance, vector_to_closest_object2);

      if (turn < 3)
      {

        
      //translation step
      double translation_distance = (max(min(closest_distance,vector_to_closest_object2.norm()),GAP)- GAP)*PERCENT_TRANSLATION;
      if (i ==closest_mesh)
        translation_distance *= 0.5;

      Eigen::Vector3d movement_direction = vector_to_closest_object.normalized();
      meshes[i].translate(-movement_direction*translation_distance*meshes[i].getMass()/total_mass);
      if (i!=closest_mesh)
        meshes[closest_mesh].translate(movement_direction*translation_distance*meshes[closest_mesh].getMass()/total_mass);
      else
        meshes[i].translate(-movement_direction*translation_distance*meshes[closest_mesh].getMass()/total_mass);



//////////
      //find closest distance
       vector_to_closest_object = meshes[i].smallestVectorToCube(cube_size);
       closest_distance = vector_to_closest_object.norm();

       closest_mesh = i;
      for (int j = 0; j < meshCount; ++j)
        if (i != j)
        {
          meshes[i].updateMinDistance(&meshes[j], cube_size, closest_distance, vector_to_closest_object);
          closest_mesh = j;
        }

      closest_distance = vector_to_closest_object.norm();

      //find closest distance from other object
       vector_to_closest_object2 = meshes[closest_mesh].smallestVectorToCube(cube_size);
      for (int j = 0; j < meshCount; ++j)
        if (closest_mesh != j)
          meshes[closest_mesh].updateMinDistance(&meshes[j], cube_size, closest_distance, vector_to_closest_object2);
      }

//////////

      else
      {
      //rotation distance
      total_mass = meshes[i].getMass() + meshes[closest_mesh].getMass();
      double rotation_distance = (max(min(closest_distance,vector_to_closest_object2.norm()),GAP)- GAP)*PERCENT_ROTATION;
      meshes[i].rotateLessThan(rotation_distance*meshes[i].getMass()/total_mass,vector_to_closest_object);
      if (i!=closest_mesh)
        meshes[closest_mesh].rotateLessThan(-rotation_distance*meshes[closest_mesh].getMass()/total_mass,vector_to_closest_object2);
      else
        meshes[i].rotateLessThan(rotation_distance*meshes[i].getMass()/total_mass,vector_to_closest_object);

     // assert(!(closest_distance < GAP));
      }  
      if (closest_distance > GAP)
      {
        still_moving = true;
      }      
      cout << meshes[i].getMass()/total_mass << endl;
      cout << i << " min distance:" << closest_distance ;   
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