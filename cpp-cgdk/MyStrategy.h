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
struct Point;

class MyStrategy : public Strategy {
public:
    MyStrategy();
    vector<Point> way;
    size_t curr_index;
    vector<double> speedVector;
    bool isStunned;
    int removeStun;
    void move(const model::Car& self, const model::World& world, const model::Game& game, model::Move& move);
};

#endif
