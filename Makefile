CPP = g++
LD = g++
EIGEN_DIR = /usr/local/include/eigen3

CPPFLAGS = -O0 -Wall -I$(EIGEN_DIR)  -DEIGEN3_INCLUDE_DIR=$(EIGEN_DIR) -DCGAL_EIGEN3_ENABLED -DEIGEN_PERMANENTLY_DISABLE_STUPID_WARNINGS -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET -fPIC -fno-inline
LDFLAGS = -Wall -Werror -fPIC 

INCLUDE = -Iinclude/
LIB =
OBJS = obj/main.o obj/mesh.o obj/quadtree.o
TARGET = anneal 

anneal: $(OBJS)
	$(LD) $(OBJS) $(LDFLAGS) $(LIB) -o $(TARGET)

obj/main.o: src/main.cpp
	$(CPP) -c $(CPPFLAGS) $(INCLUDE) src/main.cpp -o obj/main.o

obj/quadtree.o: src/quadtree.cpp
	$(CPP) -c $(CPPFLAGS) $(INCLUDE) src/quadtree.cpp -o obj/quadtree.o

obj/mesh.o: src/mesh.cpp
	$(CPP) -c $(CPPFLAGS) $(INCLUDE) src/mesh.cpp -o obj/mesh.o

clean:
	rm obj/*.o anneal