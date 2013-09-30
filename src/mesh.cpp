#include "mesh.h"

Mesh::Mesh(){
  vertex_count = 0;
  face_count = 0;
  vertexBuffer = NULL;
  indexBuffer = NULL;

  current_center = Eigen::Vector3f(0,0,0);
  prev_center = Eigen::Vector3f(0,0,0);
  boundingSphere = 0;

  quad_tree_start = NULL;;
}

void Mesh::moveToOrigin(){
  if (!vertexBuffer)
    return;
  float x,y,z,a,b,c;
  boundingBox(x,a,y,b,z,c);
  move(Eigen::Vector3f(-x,-y,-z));
}

void Mesh::boundingBox(float &x, float &x_max, float &y, float &y_max, float &z, float &z_max){
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
void Mesh::rotateLessThan(float max_rotation, Eigen::Vector3f& vector_to_closest_object){
  Eigen::Vector3f midPoint = (current_center+prev_center)*0.5;
  Eigen::Vector3f rotation_axis = (-vector_to_closest_object).cross(current_center-prev_center);
  if (rotation_axis == Eigen::Vector3f(0,0,0))
    return;
  else
    rotation_axis.normalize();
  float max_distance_to_midpoint = (current_center-midPoint).norm() + boundingSphere;
  float angle = asin(min(max_rotation,max_distance_to_midpoint)/max_distance_to_midpoint)*2;

  rotate(Eigen::AngleAxisf(angle, rotation_axis)*Eigen::Scaling(1.0f), midPoint);

}
void Mesh::rotate(Eigen::Matrix3f rotation, Eigen::Vector3f about)
{
  for (int i = 0; i < vertex_count; ++i)
    vertexBuffer[i] = rotation*(vertexBuffer[i]-about)+about ;
}

void Mesh::move(Eigen::Vector3f translation)
{
  prev_center = current_center;
  current_center+= translation;

  for (int i = 0; i < vertex_count; ++i)
    vertexBuffer[i] += translation;
}
void Mesh::updateMinDistance(Mesh* secondMesh, float& distance, Eigen::Vector3f& vector_to_closest_object){
  
  Eigen::Vector3f dist = quad_tree_start->shortestDistanceTo(secondMesh->quad_tree_start);

  float new_distance = dist.norm();
  if (new_distance < distance)
  {
    distance = new_distance;
    vector_to_closest_object = dist;
  }

/*
  
  float dist_squared = distance* distance;

  for (int i = 0; i < vertex_count; ++i)
    for (int j = 0; j < secondMesh->vertex_count; ++j)
    {
      float new_distance = (vertexBuffer[i]-secondMesh->vertexBuffer[j]).squaredNorm();
      if (new_distance < dist_squared)
      {
        distance = new_distance;
        dist_squared = distance*distance;
        vector_to_closest_object = (secondMesh->vertexBuffer[j]-vertexBuffer[i]);
      }
    }
*/

}

void Mesh::boundingBoxSize(float &j, float &k, float &l){
  if (!vertexBuffer)
    return;

  float x,y,z,a,b,c;
  boundingBox(x,a,y,b,z,c);

  j = a-x;
  k = b-y;
  l = c-z;
}

void updateMinVector(float distance, Eigen::Vector3f vector, float& min_distance, Eigen::Vector3f& new_vector)
{
  if (distance < min_distance)
  {
    min_distance = distance;
    new_vector = vector*min_distance;
  }
}

Eigen::Vector3f Mesh::smallestVectorToCube(float cube_size){
  float x,y,z,a,b,c;
  boundingBox(x,a,y,b,z,c);

  Eigen::Vector3f shortest_vector(-x,0,0);
  float min_distance = x;
  
  updateMinVector(y, Eigen::Vector3f(0,-1,0), min_distance, shortest_vector);
  updateMinVector(z, Eigen::Vector3f(0,0,-1), min_distance, shortest_vector);

  updateMinVector(cube_size-a, Eigen::Vector3f(1,0,0), min_distance, shortest_vector);
  updateMinVector(cube_size-b, Eigen::Vector3f(0,1,0), min_distance, shortest_vector);
  updateMinVector(cube_size-c, Eigen::Vector3f(0,0,1), min_distance, shortest_vector);

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
  out_mesh->vertexBuffer = new Eigen::Vector3f[out_mesh->vertex_count];
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
      out_mesh->vertexBuffer[out_mesh->vertex_count] = Eigen::Vector3f(x, y, z);
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

  float x,y,z,a,b,c;
  out_mesh->boundingBox(x,a,y,b,z,c);

  out_mesh->current_center = (Eigen::Vector3f(x,y,z)+Eigen::Vector3f(a,b,c))*0.5;
  out_mesh->prev_center = Eigen::Vector3f(0,0,-1) + out_mesh->current_center;
  out_mesh->boundingSphere = (out_mesh->current_center-Eigen::Vector3f(x,y,z)).norm();

  std::vector<Eigen::Vector3f> vertex_list;
  for (int i = 0; i < out_mesh->vertex_count; ++i)
    vertex_list.push_back(out_mesh->vertexBuffer[i]);

  out_mesh->quad_tree_start = new QuadtreeNode(vertex_list);
/////////////////////////////////////////////////////////
}


void Mesh::write(char* out_file){
  ofstream stream(out_file);

  for (int i = 0; i < vertex_count; ++i)
    stream << "v " << vertexBuffer[i](0) << " " << vertexBuffer[i](1) << " "<< vertexBuffer[i](2) << endl;

  for (int i = 0; i < face_count; ++i)
    stream << "f " << indexBuffer[3*i] << " " << indexBuffer[3*i+1] << " "<< indexBuffer[3*i+2] << endl;

  stream.close();
}
Mesh::~Mesh(){
  if (quad_tree_start){
    delete[] quad_tree_start;
    quad_tree_start = NULL;
  }
  if (vertexBuffer){
    delete[] vertexBuffer;
    vertexBuffer = NULL;
  }
  if (indexBuffer){
    delete[] indexBuffer;
    indexBuffer = NULL;
  }
}
