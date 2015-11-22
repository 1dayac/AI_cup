#include "MyStrategy.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES
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
        return y_ < other.y_ or (y_ == other.y_ and x_ < other.x_);
    }

    int x_;
    int y_;
};

typedef pair<Point, Point> Edge;

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

bool Adjacent(Edge e1, Edge e2) {
    if(e1.second == e2.first) {
        return true;
    }
    return false;
}

bool IsStraightLine(Edge e1, Edge e2) {
    return (abs(e1.first.x_ - e2.second.x_) == 0 and abs(e1.first.y_ - e2.second.y_) == 2) ||
           (abs(e1.first.x_ - e2.second.x_) == 2 and abs(e1.first.y_ - e2.second.y_) == 0);
}


bool IsTurn(Edge e1, Edge e2) {
    return !IsStraightLine(e1, e2);
}

struct EdgeBasedGraph {
    set<Edge> edge_graph_vertices_;
    map<pair<Edge, Edge>, int> edge_graph_edges_;
    EdgeBasedGraph(set<Edge>& edge_graph_vertices, map<pair<Edge, Edge>, int>& edge_graph_edges)
    : edge_graph_vertices_(edge_graph_vertices), edge_graph_edges_(edge_graph_edges)
    {  }
};

EdgeBasedGraph ConvertToEdgeBasedGraph(vector<vector<TileType>>& my_world) {
    set<Edge> edge_graph_vertices;
    map<pair<Edge, Edge>, int> edge_graph_edges;
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
        for(auto e2 : edge_graph_vertices) {
            if(Adjacent(e, e2)) {
                if(IsTurn(e, e2)) {
                    edge_graph_edges[make_pair(e,e2)] = 3;
                }

                if(IsStraightLine(e, e2)) {
                    edge_graph_edges[make_pair(e,e2)] = 1;
                }
            }
        }
    }

    return EdgeBasedGraph(edge_graph_vertices, edge_graph_edges);
}

void AddStartAndEndNodes(EdgeBasedGraph& graph, Point startPoint) {
    vector<Edge> start_vertices;
    vector<pair<Edge, Edge>> start_edges;
    vector<Edge> end_vertices;
    vector<pair<Edge, Edge>> end_edges;

    for(auto e : graph.edge_graph_vertices_) {
        start_vertices.push_back(Edge(make_pair(Point(-1,-1), e.first)));
        end_vertices.push_back(Edge(make_pair(e.second, Point(-1,-1))));

        start_edges.push_back(make_pair(Edge(make_pair(Point(-1,-1), e.first)),e));
        end_edges.push_back(make_pair(e, Edge(make_pair(e.second, Point(-1,-1)))));
    }

    for(auto e : start_edges) {
        graph.edge_graph_edges_[e] = 0;
    }

    for(auto e : end_edges) {
        graph.edge_graph_edges_[e] = 0;
    }

    for(auto e : start_vertices) {
        graph.edge_graph_vertices_.insert(e);
    }

    for(auto e : end_vertices) {
        graph.edge_graph_vertices_.insert(e);
    }
}

Point CurrentTile(double x, double y) {
    return Point(x/800, y/800);
}

Point CurrentTile(const Car& self) {
    return CurrentTile(self.getX(), self.getY());
}

vector<Point> PathFromPreds(map<Edge, Edge> preds, Point start, Point end) {
    vector<Point> answer;

    auto tempStart = Edge(make_pair(start, Point(-1, -1)));

    for(; tempStart.second != start; tempStart = preds[tempStart]) {
        answer.push_back(tempStart.first);
    }
    reverse(answer.begin(), answer.end());
    return answer;
}

vector<Point> Dijkstra(EdgeBasedGraph graph, Point startPoint, Point endPoint, Direction direction) {
    if(direction == RIGHT) {
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_ + 1, startPoint.y_)))] = 0;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_ - 1, startPoint.y_)))] = 3;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_, startPoint.y_ + 1)))] = 2;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_, startPoint.y_ - 1)))] = 2;
    }

    if(direction == LEFT) {
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_ + 1, startPoint.y_)))] = 3;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_ - 1, startPoint.y_)))] = 0;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_, startPoint.y_ + 1)))] = 2;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_, startPoint.y_ - 1)))] = 2;
    }
    if(direction == UP) {
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_ + 1, startPoint.y_)))] = 2;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_ - 1, startPoint.y_)))] = 2;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_, startPoint.y_ + 1)))] = 3;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_, startPoint.y_ - 1)))] = 0;
    }
    if(direction == DOWN) {
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_ + 1, startPoint.y_)))] = 2;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_ - 1, startPoint.y_)))] = 2;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_, startPoint.y_ + 1)))] = 0;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_, startPoint.y_ - 1)))] = 3;
    }

    priority_queue<pair<int, Edge>> q;
    map<Edge, int> dist;
    for(auto e : graph.edge_graph_vertices_) {
        dist[e] = INFINITY;
    }

    map<Edge, Edge> pred;
    q.push(make_pair(0, make_pair(Point(-1,-1), startPoint)));
    while(!q.empty()) {
        auto node = q.top();
        q.pop();
        int weight = node.first;
        Edge target = node.second;
        if(weight > dist[target]) {
            continue;
        }

        if(target.first == endPoint && target.second == Point(-1, -1)) {
            return PathFromPreds(pred, endPoint, startPoint);
        }

        //add all neighbours and end points
        Edge end = make_pair(target.second, Point(-1,-1));

        if(graph.edge_graph_vertices_.count(end) > 0 and dist[target] + graph.edge_graph_edges_[make_pair(target, end)] < dist[end] ) {
            dist[end] = dist[target] + graph.edge_graph_edges_[make_pair(target, end)];
            pred[end] = target;
            q.push(make_pair(-dist[end], end));
        }

        Edge left = make_pair(target.second, Point(target.second.x_ - 1,target.second.y_));

        if(graph.edge_graph_vertices_.count(left) > 0 and dist[target] + graph.edge_graph_edges_[make_pair(target, left)] < dist[left] ) {
            dist[left] = dist[target] + graph.edge_graph_edges_[make_pair(target, left)];
            pred[left] = target;
            q.push(make_pair(-dist[left], left));
        }

        Edge right = make_pair(target.second, Point(target.second.x_ + 1,target.second.y_));

        if(graph.edge_graph_vertices_.count(right) > 0 and dist[target] + graph.edge_graph_edges_[make_pair(target, right)] < dist[right] ) {
            dist[right] = dist[target] + graph.edge_graph_edges_[make_pair(target, right)];
            pred[right] = target;
            q.push(make_pair(-dist[right], right));
        }

        Edge up = make_pair(target.second, Point(target.second.x_,target.second.y_ - 1));

        if(graph.edge_graph_vertices_.count(up) > 0 and dist[target] + graph.edge_graph_edges_[make_pair(target, up)] < dist[up] ) {
            dist[up] = dist[target] + graph.edge_graph_edges_[make_pair(target, up)];
            pred[up] = target;
            q.push(make_pair(-dist[up], up));
        }

        Edge down = make_pair(target.second, Point(target.second.x_,target.second.y_ + 1));

        if(graph.edge_graph_vertices_.count(down) > 0 and dist[target] + graph.edge_graph_edges_[make_pair(target, down)] < dist[down] ) {
            dist[down] = dist[target] + graph.edge_graph_edges_[make_pair(target, down)];
            pred[down] = target;
            q.push(make_pair(-dist[down], down));
        }
    }
}


Direction GetDirection(Point from, Point to) {
    if(from.y_ == to.y_) {
        if(from.x_ == to.x_ + 1) {
            return RIGHT;
        } else if (from.x_ == to.x_ - 1) {
            return LEFT;
        }
    }

    if(from.x_ == to.x_) {
        if(from.y_ == to.y_ + 1) {
            return DOWN;
        } else if (from.y_ == to.y_ - 1) {
            return UP;
        }
    }
    return UP;
}

vector<Point> bestPath(const Car& self, const World& world, const Game& game) {
    auto space = world.getTilesXY();
    Point startPoint = CurrentTile(self);
    Direction startDirection = world.getStartingDirection();
    auto graph = ConvertToEdgeBasedGraph(space);
    AddStartAndEndNodes(graph, startPoint);
    vector<Point> final_answer;
    for(int i = 0; i < world.getWaypoints().size() - 1; ++i) {
        Point from = Point(world.getWaypoints()[i][0], world.getWaypoints()[i][1]);
        Point to = Point(world.getWaypoints()[i+1][0], world.getWaypoints()[i+1][1]);
        Direction direction = i == 0 ? world.getStartingDirection() : GetDirection(final_answer[final_answer.size() - 2], final_answer[final_answer.size() - 1]);
        vector<Point> answer = Dijkstra(graph, from, to, direction);
        for(auto elem : answer) {
            if(final_answer.size() != 0 && elem != final_answer.back()) {
                final_answer.push_back(elem);
            }
            if(final_answer.size() == 0) {
                final_answer.push_back(elem);
            }
         }
    }
    return final_answer;
}

void MyStrategy::move(const Car& self, const World& world, const Game& game, Move& move) {
    int xCoord = self.getNextWaypointX()  * game.getTrackTileSize() + 0.5 * game.getTrackTileSize();
    int yCoord = self.getNextWaypointY() * game.getTrackTileSize() + 0.5 * game.getTrackTileSize();
    double angle = self.getAngleTo(xCoord, yCoord);
    double angleCar = self.getAngle();
    bestPath(self, world, game);
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
