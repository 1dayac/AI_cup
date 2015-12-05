#pragma once

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <set>
#include <map>
#include <queue>
using namespace model;
using namespace std;

struct Point {

    Point(int x = -100, int y = -100)
            : x_(x), y_(y) {

    };

    bool operator==(const Point &other) const {
        return x_ == other.x_ && y_ == other.y_;
    }

    bool operator!=(const Point &other) const {
        return !(*this == other);
    }


    bool operator<(const Point &other) const {
        return y_ < other.y_ || (y_ == other.y_ && x_ < other.x_);
    }

    void Print() {
        cout << x_ << " " << y_ << endl;
    }

    int x_;
    int y_;
};


typedef pair<Point, Point> Edge;

struct EdgeBasedGraph {
    set<Edge> edge_graph_vertices_;
    map<pair<Edge, Edge>, int> edge_graph_edges_;
    EdgeBasedGraph(set<Edge>& edge_graph_vertices, map<pair<Edge, Edge>, int>& edge_graph_edges)
            : edge_graph_vertices_(edge_graph_vertices), edge_graph_edges_(edge_graph_edges)
    {  }

    EdgeBasedGraph()
    {}
};

class MyStrategy : public Strategy {
public:
    MyStrategy();
    vector<Point> way;
    vector<Point> additional_way;
    size_t curr_index;
    size_t additional_index;
    vector<double> speedVector;
    bool isStunned;
    bool lostWay;
    int removeStun;
	double getWheelTurnW(double alpha, const Car& self, const Game& game);
    void move(const model::Car& self, const model::World& world, const model::Game& game, model::Move& move);
    vector<Point> bestPath(const Car& self, const World& world, const Game& game);
    void SetInitialCoords(int& x, int& y, const Car& self);
    EdgeBasedGraph graph;
};

#endif
