#include "mesh.h"

Mesh::Mesh(){
  vertex_count = 0;
  face_count = 0;
  vertexBuffer = NULL;
  indexBuffer = NULL;

  current_center = Eigen::Vector3d(0,0,0);
  prev_center = Eigen::Vector3d(0,0,0);
  boundingSphere = 0;

}

void Mesh::moveToOrigin(){
  if (!vertexBuffer)
    return;
  double x,y,z,a,b,c;
  boundingBox(x,a,y,b,z,c);
  move(Eigen::Vector3d(-x,-y,-z));
}

void Mesh::boundingBox(double &x, double &x_max, double &y, double &y_max, double &z, double &z_max){
  if (!vertexBuffer)
    return;

  x = vertexBuffer[0](0);   x_max = vertexBuffer[0](0);
  y = vertexBuffer[0](1);   y_max = vertexBuffer[0](1);
  z = vertexBuffer[0](2);   z_max = vertexBuffer[0](2);

  for (int i = 0; i < vertex_count; ++i)
  {
    if (vertexBuffer[i](0) < x)
      x = vertexBuffer[i](0);
    if (vertexBuffer[i](0) > x_max)
      x_max = vertexBuffer[i](0);
    if (vertexBuffer[i](1) < y)
      y = vertexBuffer[i](1);
    if (vertexBuffer[i](1) > y_max)
      y_max = vertexBuffer[i](1);
    if (vertexBuffer[i](2) < z)
      z = vertexBuffer[i](2);
    if (vertexBuffer[i](2) > z_max)
      z_max = vertexBuffer[i](2);

  }

}
void Mesh::concatenate(Mesh* meshArray, int mesh_count, Mesh* out_mesh){
    int face_count = 0;
    int vertex_count = 0;
    int* buffer_sizes = new int[mesh_count];

    for (int i = 0; i < mesh_count; ++i)
    {
      buffer_sizes[i] = vertex_count;
      face_count += meshArray[i].face_count;
      vertex_count += meshArray[i].vertex_count;
    }

    out_mesh->face_count = face_count;
    out_mesh->vertex_count = vertex_count;
    out_mesh->vertexBuffer = new Eigen::Vector3d[vertex_count];
    out_mesh->indexBuffer = new int[face_count*3];

    Eigen::Vector3d* vertex_pointer = out_mesh->vertexBuffer;
    int* index_pointer = out_mesh->indexBuffer;

    for (int i = 0; i < mesh_count; ++i)
    {
      for (int j = 0; j < meshArray[i].vertex_count; ++j)
        vertex_pointer[j] = meshArray[i].vertexBuffer[j];

      for (int j = 0; j < meshArray[i].face_count*3; ++j)
        index_pointer[j] = meshArray[i].indexBuffer[j] + buffer_sizes[i];

      vertex_pointer += meshArray[i].vertex_count;
      index_pointer+= meshArray[i].face_count*3;      
    }

}


void Mesh::rotateLessThan(double max_rotation, Eigen::Vector3d& vector_to_closest_object){

  Eigen::Vector3d midPoint = (current_center+prev_center)*0.5;
  Eigen::Vector3d rotation_axis = (vector_to_closest_object).cross(prev_closest_point-current_closest_point);

  if (rotation_axis == Eigen::Vector3d(0,0,0))
    return;
  else
    rotation_axis.normalize();
  double max_distance_to_midpoint = (current_center-midPoint).norm() + boundingSphere;

  double angle = asin(max_rotation/(2*max_distance_to_midpoint))*2;

  if (angle == angle)
    rotate(Eigen::AngleAxisd(angle, rotation_axis)*Eigen::Scaling(1.0), midPoint);

}
void Mesh::rotate(Eigen::Matrix3d rotation, Eigen::Vector3d about)
{
  for (int i = 0; i < vertex_count; ++i)
    vertexBuffer[i] = rotation*(vertexBuffer[i]-about)+about ;
}

void Mesh::translate(Eigen::Vector3d translation)
{
  prev_center += translation;
  current_center+= translation;

  for (int i = 0; i < vertex_count; ++i)
    vertexBuffer[i] += translation;
}

void Mesh::move(Eigen::Vector3d translation)
{
  prev_center = current_center;
  current_center+= translation;

  for (int i = 0; i < vertex_count; ++i)
    vertexBuffer[i] += translation;
}
void Mesh::buildQuadtree(QuadtreeNode** out_tree, double cube_size){
  std::vector<Triangle*> vertex_list;
  for (int i = 0; i < face_count; ++i)
  {
    vertex_list.push_back(new Triangle(vertexBuffer[indexBuffer[3*i]-1],vertexBuffer[indexBuffer[3*i+1]-1],
      vertexBuffer[indexBuffer[3*i+2]-1]));
  }

  *out_tree = new QuadtreeNode(vertex_list, Eigen::Vector3d(0,0,0), Eigen::Vector3d(cube_size, cube_size, cube_size));

}

void Mesh::update()
{
  prev_closest_point = current_closest_point;
}

void Mesh::updateMinDistance(Mesh* secondMesh, double cube_size, double& distance, Eigen::Vector3d& vector_to_closest_object){

  QuadtreeNode* tree1; QuadtreeNode* tree2;
  buildQuadtree(&tree1, cube_size);
  secondMesh->buildQuadtree(&tree2, cube_size);
  tree2->updateShortestDistanceTo(tree1, vector_to_closest_object, current_closest_point);
  delete tree1;
  delete tree2;
 /*
  double dist_squared = distance* distance;

  for (int i = 0; i < vertex_count; ++i)
    for (int j = 0; j < secondMesh->vertex_count; ++j)
    {
      double new_distance = (vertexBuffer[i]-secondMesh->vertexBuffer[j]).squaredNorm();
      if (new_distance < dist_squared)
      {
        distance = new_distance;
        dist_squared = distance*distance;
        vector_to_closest_object = (secondMesh->vertexBuffer[j]-vertexBuffer[i]);
      }
    }
*/
}

void Mesh::boundingBoxSize(double &j, double &k, double &l){
  if (!vertexBuffer)
    return;

  double x,y,z,a,b,c;
  boundingBox(x,a,y,b,z,c);

  j = a-x;
  k = b-y;
  l = c-z;
}

void updateMinVector(double distance, Eigen::Vector3d vector, double& min_distance, Eigen::Vector3d& new_vector)
{
  if (distance < min_distance)
  {
    min_distance = distance;
    new_vector = vector*min_distance;
  }
}

Eigen::Vector3d Mesh::smallestVectorToCube(double cube_size){
  double x,y,z,a,b,c;
  boundingBox(x,a,y,b,z,c);

  Eigen::Vector3d shortest_vector(-x,0,0);
  double min_distance = x;

  updateMinVector(y, Eigen::Vector3d(0,-1,0), min_distance, shortest_vector);
  updateMinVector(z, Eigen::Vector3d(0,0,-1), min_distance, shortest_vector);

  updateMinVector(cube_size-a, Eigen::Vector3d(1,0,0), min_distance, shortest_vector);
  updateMinVector(cube_size-b, Eigen::Vector3d(0,1,0), min_distance, shortest_vector);
  updateMinVector(cube_size-c, Eigen::Vector3d(0,0,1), min_distance, shortest_vector);

  float greatest_dist = vertexBuffer[0].dot(-shortest_vector);
  for (int i = 0; i < vertex_count; ++i)
  {
    if (vertexBuffer[i].dot(-shortest_vector) > greatest_dist)
    {
      current_closest_point = vertexBuffer[i]+ shortest_vector;
      greatest_dist = vertexBuffer[i].dot(-shortest_vector);
    }
  }

  return shortest_vector;
}


void getObjFileLength(char* obj_file, int* face_count, int* vertex_count){
  std::ifstream obj_stream(obj_file);
  string str;

  int i = 0;

  getline(obj_stream, str, 'v');
  //read each obj vertex
  while(getline(obj_stream, str, 'v')){
    i++;
  }
  *vertex_count = i;
  obj_stream.close();


  obj_stream.open(obj_file);
  i = 0;
  getline(obj_stream, str, 'f');
  //read each obj index
  while(getline(obj_stream, str, 'f')){
    i++;
  }
  *face_count = i*2; //to account for possible quads
  obj_stream.close();
}

void Mesh::meshFromFile(char* filename, Mesh* out_mesh){
  //initalizing mesh
  getObjFileLength(filename, &out_mesh->face_count, &out_mesh->vertex_count);
  out_mesh->vertexBuffer = new Eigen::Vector3d[out_mesh->vertex_count];
  out_mesh->indexBuffer = new int[out_mesh->face_count*3];

  out_mesh->face_count = 0;
  out_mesh->vertex_count = 0;

  // Open file
  FILE *fp;
  if (!(fp = fopen(filename, "r"))) {
    cerr << "Unable to open file " << filename;
  }

  // Read body
  char buffer[1024];
  int line_count = 0;
  int triangle_count = 0;
  while (fgets(buffer, 1023, fp)) {
    // Increment line counter
    line_count++;

    // Skip white space
    char *bufferp = buffer;
    while (isspace(*bufferp)) bufferp++;

    // Skip blank lines and comments
    if (*bufferp == '#') continue;
    if (*bufferp == '\0') continue;

    // Get keyword
    char keyword[80];
    if (sscanf(bufferp, "%s", keyword) != 1) {
      cerr << "Syntax error on line " << line_count << " in file " << filename;
    }

    // Check keyword
    if (!strcmp(keyword, "v")) {
      // Read vertex coordinates
      double x, y, z;
      if (sscanf(bufferp, "%s%lf%lf%lf", keyword, &x, &y, &z) != 4) {
        cerr << "Syntax error on line " << line_count << " in file " << filename;
      }

      // Create vertex
      out_mesh->vertexBuffer[out_mesh->vertex_count] = Eigen::Vector3d(x, y, z);
      out_mesh->vertex_count++;
    }
    else if (!strcmp(keyword, "f")) {
      // Read vertex indices
      int quad = 1;
      char s1[128], s2[128], s3[128], s4[128] = { '\0' };
      if (sscanf(bufferp, "%s%s%s%s%s", keyword, s1, s2, s3, s4) != 5) {
        quad = 0;;
        if (sscanf(bufferp, "%s%s%s%s", keyword, s1, s2, s3) != 4) {
          cerr << "Syntax error on line " << line_count << " in file " << filename;
        }
      }

      // Parse vertex indices
      int i1, i2, i3, i4 = -1;
      char *p1 = strchr(s1, '/'); if (p1) *p1 = 0; i1 = atoi(s1);
      char *p2 = strchr(s2, '/'); if (p2) *p2 = 0; i2 = atoi(s2);
      char *p3 = strchr(s3, '/'); if (p3) *p3 = 0; i3 = atoi(s3);
      if (quad) {
        char *p4 = strchr(s4, '/'); if (p4) *p4 = 0; i4 = atoi(s4);
      }

      if (quad){
        out_mesh->indexBuffer[3*out_mesh->face_count] = i1;
        out_mesh->indexBuffer[3*out_mesh->face_count+1] = i2;
        out_mesh->indexBuffer[3*out_mesh->face_count+2] = i3;
        out_mesh->face_count ++;

        out_mesh->indexBuffer[3*out_mesh->face_count] = i2;
        out_mesh->indexBuffer[3*out_mesh->face_count+1] = i4;
        out_mesh->indexBuffer[3*out_mesh->face_count+2] = i3;
        out_mesh->face_count ++;

      }
      else{ 
        out_mesh->indexBuffer[3*out_mesh->face_count] = i1;
        out_mesh->indexBuffer[3*out_mesh->face_count+1] = i2;
        out_mesh->indexBuffer[3*out_mesh->face_count+2] = i3;
        out_mesh->face_count ++;
      }


        // Increment triangle counter
        triangle_count++;
      }

      // Increment triangle counter
      triangle_count++;
    }

  // Close file
  fclose(fp);

  double x,y,z,a,b,c;
  out_mesh->boundingBox(x,a,y,b,z,c);

  out_mesh->current_center = (Eigen::Vector3d(x,y,z)+Eigen::Vector3d(a,b,c))*0.5;
  out_mesh->prev_center = Eigen::Vector3d(0,0,-1) + out_mesh->current_center;
  out_mesh->boundingSphere = (out_mesh->current_center-Eigen::Vector3d(x,y,z)).norm();

  out_mesh->current_closest_point = out_mesh->vertexBuffer[0];
  out_mesh->prev_closest_point = out_mesh->vertexBuffer[0];

/////////////////////////////////////////////////////////
}


void Mesh::write(char* out_file){
  ofstream stream(out_file);

  for (int i = 0; i < vertex_count; ++i)
    stream << "v " << vertexBuffer[i](0) << " " << vertexBuffer[i](1) << " "<< vertexBuffer[i](2) << endl;

  for (int i = 0; i < face_count; ++i)
    stream << "f " << indexBuffer[3*i] << " " << indexBuffer[3*i+1] << " "<< indexBuffer[3*i+2] << endl;

/*  stream << "solid mesh" << endl;

  for (int i = 0; i < face_count; ++i)
  {
    stream << "facet normal 0 0 0" << endl;
    stream << "outer loop" << endl;
      for (int j = 0; j < 3; ++j)
        stream << "vertex " << vertexBuffer[indexBuffer[3*i+j]+1](0) << " " <<
             vertexBuffer[indexBuffer[3*i+j]+1](1) << " "<< vertexBuffer[indexBuffer[3*i+j]+1](2) << endl;
    stream << "end loop" << endl;
    stream << "end facet" << endl;
  }

  stream << "endsolid mesh" << endl*/
  stream.close();
}
Mesh::~Mesh(){
  if (vertexBuffer){
    delete[] vertexBuffer;
    vertexBuffer = NULL;
  }
  if (indexBuffer){
    delete[] indexBuffer;
    indexBuffer = NULL;
  }
}
