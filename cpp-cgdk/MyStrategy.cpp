#include "MyStrategy.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace model;
using namespace std;

struct Point {

    Point(int x, int y)
    : x_(x), y_(y) {

    };
    int x_;
    int y_;
};


Point CurrentTile(double x, double y) {
    return Point(x/800, y/800);
}

Point CurrentTile(const Car& self) {
    return CurrentTile(self.getX(), self.getY());
}

vector<Point> bestPath(const Car& self, const World& world, const Game& game) {
    auto space = world.getTilesXY();
    Point startPoint = CurrentTile(self);
    Direction startDirection = world.getStartingDirection();
    
}

void MyStrategy::move(const Car& self, const World& world, const Game& game, Move& move) {
    int xCoord = self.getNextWaypointX()  * game.getTrackTileSize() + 0.5 * game.getTrackTileSize();
    int yCoord = self.getNextWaypointY() * game.getTrackTileSize() + 0.5 * game.getTrackTileSize();
    double angle = self.getAngleTo(xCoord, yCoord);
    double angleCar = self.getAngle();
    cout << xCoord << " " << yCoord << " " << angle << " " << angleCar << endl;
    if(angleCar > angle) {
        move.setWheelTurn(-0.8);
    } else {
        move.setWheelTurn(0.8);
    }
    move.setEnginePower(1.0);
    move.setThrowProjectile(true);
    move.setSpillOil(true);

    if (world.getTick() > game.getInitialFreezeDurationTicks()) {
        move.setUseNitro(true);
    }
}

MyStrategy::MyStrategy() { }
