#include "Box.h"

// counterclockwise is positive
Box::Box(Vector2f origin, float sideLenX, float sideLenY, float theta) : sideX(sideLenX, 0), sideY(0, sideLenY), rotation(theta) {
    this->origin = origin;

    this->sideX = rotation*this->sideX;
    this->sideY = rotation*this->sideY;
}

Box::Box(Vector2f point1, Vector2f point2, float buffer) : 
sideX(2*buffer, 0), 
sideY(0, 2*buffer + this->distanceBetween(point1, point2)), 
rotation(atan(this->twoPointSlope(point1, point2))) 
{
    Vector2f vecBetween = point2 - point1;
    this->sideX = this->rotation*this->sideX;
    this->sideY = this->rotation*this->sideY;

    this->origin = point1 - (.5*sideX) - (buffer*this->sideY.normalized());
}

// returns slope, y intercept
LinearLine Box::findLinearEquation(BoxSide side) {
    LinearLine line;
    Vector2f point1(0,0,0);
    Vector2f point2(0,0,0);
    if (side == BOTTOMX) {
        point1 = this->origin;
        point2 = this->origin + this->sideX;
    } else if (side  == TOPX) {
        point1 = this->origin + this->sideY;
        point2 = this->origin + this->sideX + this->sideY;
    } else if (side == LEFTY) {
        point1 = this->origin;
        point2 = this->origin + this->sideY;
    } else if (side == RIGHTY) {
        point1 = this->origin + this->sideX;
        point2 = this->origin + this->sideX + this->sideY;
    } else {
        Serial.println("Invalid box side");
    }

    line.slope = this->twoPointSlope(point1, point2);
    line.yInt = this->findYInt(line.slope, point1);

    return line;
}

void Box::printBox() {
    Serial.printf("sideX: x:%f, y:%f\n", this->sideX(0), this->sideX(1));
    Serial.printf("sideY: x:%f, y:%f\n", this->sideY(0), this->sideY(1));
    Serial.printf("origin: x:%f, y:%f\n", this->origin(0), this->origin(1));
    Serial.printf("rotation: %f\n", this->rotation.angle());
}








// PRIVATE FUNCTIONS


float Box::distanceBetween(Vector2f origin, Vector2f point) {
  Vector2f difference = point - origin;
  return difference.norm();
}

float Box::twoPointSlope(Vector2f point1, Vector2f point2) {
    return (point2(0) - point1(0)) / (point2(1) - point1(1));
}

float Box::findYInt(float slope, Vector2f point) {
    return point(1) - (slope*point(0));
}

