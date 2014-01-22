#include<bfs3d/BFS_3D.h>

#define convert_x(x) (x+.75)/.02
#define convert_y(y) (y+1.25)/.02
#define convert_z(z) (z+.1)/.02

int main(int argc, char** argv){
    sbpl_arm_planner::BFS_3D bfs(150,150,110);
    //bfs.reset();
    //bfs.run(convert_x(0.425),convert_y(-0.300),convert_z(0.802));
  //for(int i=0; i<100; i++){
  while(1){
    bfs.reset();
    bfs.run(convert_x(0.3),convert_y(0.111),convert_z(1.348));
    bfs.stop();
    bfs.reset();
    bfs.run(convert_x(0.292),convert_y(0.031),convert_z(1.348));
    bfs.stop();
  }
}
