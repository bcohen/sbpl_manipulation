#include <bfs3d/BFS_Util.h>
#include <cstdio>
#include <stdlib.h>
#include <iostream>

using namespace std;

//#include <distance_field/propagation_distance_field.h>

/*
void setDistanceField(BFS_3D &bfs, const distance_field::PropagationDistanceField* df, int radius) {
    for (int z = 0; z < dim_z - 2; z++)
        for (int y = 0; y < dim_y - 2; y++)
            for (int x = 0; x < dim_x - 2; x++)
                if (df_->getDistanceFromCell(x,y,z) <= radius)
                    bfs.setWall(z + 1, y + 1, x + 1);
    }
}
*/

int getMin(int* numbers, int n) {
  int min = 0x7FFFFFFF;
  for (int i = 0; i < n; i++)
    if (numbers[i] < min)
      min = numbers[i];
  return min;
}

void getMinNeighbor(sbpl_arm_planner::BFS_3D* bfs, int &nodeX, int &nodeY, int &nodeZ) {
  int minDistance = 0x7FFFFFFF;
  int minX, minY, minZ;
  int a=-1, b=-1, minD=100, currentD=100;
  for (int z = -1; z <= 1; z++) {
    for (int y = -1; y <= 1; y++) {
      for (int x = -1; x <= 1; x++) {
        a++;
        if (!(x == 0 && y == 0 && z == 0) && !bfs->isWall(nodeX + x, nodeY + y, nodeZ + z)) {
          b++;
        //if ((x != 0 || y != 0 || z != 0) && !bfs->isWall(x, y, z)) {
          int currentDistance = bfs->getDistance(nodeX + x, nodeY + y, nodeZ + z);
          if (currentDistance == minDistance) {
            currentD = abs(x)+abs(y)+abs(z);
            if (currentD < minD){
              minDistance = currentDistance;
              minX = nodeX + x;
              minY = nodeY + y;
              minZ = nodeZ + z;
              minD = currentD;
            }
          }
          else if (currentDistance <= minDistance) {
            minDistance = currentDistance;
            minX = nodeX + x;
            minY = nodeY + y;
            minZ = nodeZ + z;
            minD = abs(x)+abs(y)+abs(z);
          }
        }
      }
    }
  }
  //printf("[%d %d %d] min: %d  --> [%d %d %d]   (a: %d, b: %d, minD: %d)\n", nodeX, nodeY, nodeZ, minDistance, minX, minY, minZ, a, b, minD);
  nodeX = minX;
  nodeY = minY;
  nodeZ = minZ;
}



vector<vector<int> > getShortestPath(sbpl_arm_planner::BFS_3D* bfs, int currentX, int currentY, int currentZ, int endX, int endY, int endZ) {
  vector<vector<int> > path;
  vector<int> currentNode(3,0);

  //bfs->configure(endX, endY, endZ);
  //bfs->run(endX, endY, endZ);
  //sleep(1);
  bfs->getDistance(currentX, currentY, currentZ);

  int cntr = 0;
  while ((currentX != endX || currentY != endY || currentZ != endZ) && cntr < 250) {
    getMinNeighbor(bfs, currentX, currentY, currentZ);
    currentNode[0] = currentX;
    currentNode[1] = currentY;
    currentNode[2] = currentZ;
    path.push_back(currentNode);
    cntr++;
  }

  return path;
}
