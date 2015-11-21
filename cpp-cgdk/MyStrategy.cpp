#include "MyStrategy.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <unordered_set>
#include <unordered_map>

using namespace model;
using namespace std;

struct Point {

    Point(int x, int y)
    : x_(x), y_(y) {

    };
    int x_;
    int y_;
};


bool HasHoleTop(TileType tyle) {
    return tyle == VERTICAL || tyle == LEFT_BOTTOM_CORNER || tyle == RIGHT_BOTTOM_CORNER ||
    tyle == LEFT_HEADED_T || tyle == RIGHT_HEADED_T || tyle == TOP_HEADED_T || tyle == CROSSROADS;

}

bool HasHoleBottom(TileType tyle) {
    return tyle == VERTICAL || tyle == LEFT_TOP_CORNER || tyle == RIGHT_TOP_CORNER ||
           tyle == LEFT_HEADED_T || tyle == RIGHT_HEADED_T || tyle == BOTTOM_HEADED_T || tyle == CROSSROADS;

}

bool HasHoleLeft(TileType tyle) {
    return tyle == HORIZONTAL || tyle == RIGHT_BOTTOM_CORNER || tyle == RIGHT_TOP_CORNER ||
           tyle == LEFT_HEADED_T || tyle == TOP_HEADED_T || tyle == BOTTOM_HEADED_T || tyle == CROSSROADS;

}

bool HasHoleRight(TileType tyle) {
    return tyle == HORIZONTAL || tyle == LEFT_BOTTOM_CORNER || tyle == LEFT_TOP_CORNER ||
           tyle == RIGHT_HEADED_T || tyle == TOP_HEADED_T || tyle == BOTTOM_HEADED_T || tyle == CROSSROADS;

}


void ConvertToEdgeBasedGraph(vector<vector<TileType>>& my_world) {
    typedef pair<Point, Point> Edge;
    unordered_set<Edge> edge_graph_vertices;
    unordered_map<pair<Edge, Edge>, int> edge_graph_edges;
    for(size_t i = 0; i < my_world.size(); ++i) {
       for(size_t j = 0; i < my_world.size(); ++i) {
            if(HasHoleBottom(my_world[i][j])) {
                edge_graph_vertices.insert(Edge(Point(i, j), Point(i + 1, j)));
            }
           if(HasHoleTop(my_world[i][j])) {
               edge_graph_vertices.insert(Edge(Point(i, j), Point(i - 1, j)));
           }

           if(HasHoleRight(my_world[i][j])) {
               edge_graph_vertices.insert(Edge(Point(i, j), Point(i + 1, j)));
           }

           if(HasHoleLeft(my_world[i][j])) {
               edge_graph_vertices.insert(Edge(Point(i, j), Point(i - 1, j)));
           }

       }
   }

    for(auto e : edge_graph_vertices) {

    }
}


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
