#include "Box.h"

// counterclockwise is positive
Box::Box(Vector2f origin, float sideLenX, float sideLenY, float theta) : sideX(sideLenX, 0), sideY(0, sideLenY), rotation(theta) {
    this->origin = origin;

    this->sideX = rotation*this->sideX;
    this->sideY = rotation*this->sideY;
}

// returns slope, y intercept
Vector2f Box::findLinearEquation(BoxSide side) {
    
}