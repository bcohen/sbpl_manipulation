#include <bfs3d/BFS_3D.h>

namespace sbpl_arm_planner{

#define EXPAND_NEIGHBOR(offset)                            \
    if (distance_grid[currentNode + offset] < 0) {         \
        queue[queue_tail++] = currentNode + offset;        \
        distance_grid[currentNode + offset] = currentCost; \
    }

void BFS_3D::search(int width, int planeSize, int volatile* distance_grid, int* queue, int &queue_head, int &queue_tail) {
    while (!should_stop && queue_head < queue_tail) {
        int currentNode = queue[queue_head++];
        int currentCost = distance_grid[currentNode] + 1;

        EXPAND_NEIGHBOR(-width);
        EXPAND_NEIGHBOR(1);
        EXPAND_NEIGHBOR(width);
        EXPAND_NEIGHBOR(-1);
        EXPAND_NEIGHBOR(-width-1);
        EXPAND_NEIGHBOR(-width+1);
        EXPAND_NEIGHBOR(width+1);
        EXPAND_NEIGHBOR(width-1);
        EXPAND_NEIGHBOR(planeSize);
        EXPAND_NEIGHBOR(-width+planeSize);
        EXPAND_NEIGHBOR(1+planeSize);
        EXPAND_NEIGHBOR(width+planeSize);
        EXPAND_NEIGHBOR(-1+planeSize);
        EXPAND_NEIGHBOR(-width-1+planeSize);
        EXPAND_NEIGHBOR(-width+1+planeSize);
        EXPAND_NEIGHBOR(width+1+planeSize);
        EXPAND_NEIGHBOR(width-1+planeSize);
        EXPAND_NEIGHBOR(-planeSize);
        EXPAND_NEIGHBOR(-width-planeSize);
        EXPAND_NEIGHBOR(1-planeSize);
        EXPAND_NEIGHBOR(width-planeSize);
        EXPAND_NEIGHBOR(-1-planeSize);
        EXPAND_NEIGHBOR(-width-1-planeSize);
        EXPAND_NEIGHBOR(-width+1-planeSize);
        EXPAND_NEIGHBOR(width+1-planeSize);
        EXPAND_NEIGHBOR(width-1-planeSize);
    }
    running = false;
}
}
