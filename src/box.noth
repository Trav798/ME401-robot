#ifndef BOX_H_
#define BOX_H_

#include <Arduino.h>
#undef B1
#include <Eigen.h>

using namespace Eigen;

enum BoxSide {
    BOTTOMX = 0,
    TOPX = 1,
    LEFTY = 2,
    RIGHTY = 3,
};

struct LinearLine {
    float slope;
    float yInt; 
};

class Box {
    private:
        Vector2f origin;
        Vector2f sideX;
        Vector2f sideY;
        Rotation2Df rotation;
        float distanceBetween(Vector2f origin, Vector2f point);
        float twoPointSlope(Vector2f point1, Vector2f point2);
        float findYInt(float slope, Vector2f point);
    public:
        Box(Vector2f origin, float sideLenX, float sideLenY, float rotation);
        Box(Vector2f point1, Vector2f point2, float buffer);
        LinearLine findLinearEquation(BoxSide side);
        void printBox();
};


#endif