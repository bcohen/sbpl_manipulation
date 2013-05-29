#include <BFS_3D/BFS_3D.h>
#include <distance_field/propagation_distance_field.h>

void setDistanceField(BFS_3D &bfs, const distance_field::PropagationDistanceField* df, int radius) {
    for (int z = 0; z < dim_z - 2; z++)
        for (int y = 0; y < dim_y - 2; y++)
            for (int x = 0; x < dim_x - 2; x++)
                if (df_->getDistanceFromCell(x,y,z) <= radius)
                    bfs.setWall(z + 1, y + 1, x + 1);
    }
}
