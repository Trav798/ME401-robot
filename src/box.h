#ifndef BOX_H_
#define BOX_H_

#include <Eigen.h>

using namespace Eigen;

enum BoxSide {
    BOTTOMX = 0,
    TOPX = 1,
    LEFTY = 2,
    RIGHTY = 3,
};

class Box {
    private:
        Vector2f origin;
        Vector2f sideX;
        Vector2f sideY;
        Rotation2Df rotation;
    public:
        Box(Vector2f origin, float sideLenX, float sideLenY, float rotation);
        Vector2f findLinearEquation(BoxSide side);
};


#endif