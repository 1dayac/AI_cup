#include "MyStrategy.h"
#include <math.h>
#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES


void Print(vector<Point> v) {
    for(auto e : v) {
        e.Print();
    }
}

bool needLeft(double angle) {
    if(angle < 0 ) {
        return true;
    } else {
        return false;
    }
}

double GetSpeed(const Car& self) {
    return std::sqrt(self.getSpeedX() * self.getSpeedX() + self.getSpeedY() * self.getSpeedY());
}

double GetDistance(Point a, Point b) {
    return sqrt(pow(a.x_ - b.x_, 2) + pow(a.y_ - b.y_, 2));
}

bool HasHoleTop(TileType tyle) {
    return tyle == VERTICAL || tyle == LEFT_BOTTOM_CORNER || tyle == RIGHT_BOTTOM_CORNER ||
           tyle == LEFT_HEADED_T || tyle == RIGHT_HEADED_T || tyle == TOP_HEADED_T || tyle == CROSSROADS || tyle == UNKNOWN;

}

bool HasHoleBottom(TileType tyle) {
    return tyle == VERTICAL || tyle == LEFT_TOP_CORNER || tyle == RIGHT_TOP_CORNER ||
           tyle == LEFT_HEADED_T || tyle == RIGHT_HEADED_T || tyle == BOTTOM_HEADED_T || tyle == CROSSROADS || tyle == UNKNOWN;

}

bool HasHoleLeft(TileType tyle) {
    return tyle == HORIZONTAL || tyle == RIGHT_BOTTOM_CORNER || tyle == RIGHT_TOP_CORNER ||
           tyle == LEFT_HEADED_T || tyle == TOP_HEADED_T || tyle == BOTTOM_HEADED_T || tyle == CROSSROADS || tyle == UNKNOWN;

}

bool HasHoleRight(TileType tyle) {
    return tyle == HORIZONTAL || tyle == LEFT_BOTTOM_CORNER || tyle == LEFT_TOP_CORNER ||
           tyle == RIGHT_HEADED_T || tyle == TOP_HEADED_T || tyle == BOTTOM_HEADED_T || tyle == CROSSROADS || tyle == UNKNOWN;

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


EdgeBasedGraph ConvertToEdgeBasedGraph(vector<vector<TileType>>& my_world) {
    set<Edge> edge_graph_vertices;
    map<pair<Edge, Edge>, int> edge_graph_edges;
    for(size_t i = 0; i < my_world.size(); ++i) {
        for(size_t j = 0; j < my_world[i].size(); ++j) {
            if(HasHoleBottom(my_world[i][j])) {
                edge_graph_vertices.insert(Edge(Point(i, j), Point(i, j + 1)));
            }
            if(HasHoleTop(my_world[i][j])) {
                edge_graph_vertices.insert(Edge(Point(i, j), Point(i, j - 1)));
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
                    edge_graph_edges[make_pair(e,e2)] = 1.95;
                }

                if(IsStraightLine(e, e2)) {
                    edge_graph_edges[make_pair(e,e2)] = 2;
                }
            }
        }
    }

    return EdgeBasedGraph(edge_graph_vertices, edge_graph_edges);
}

Direction GetSomeDirection(const Car& self) {
    if(abs(self.getAngle()) <= PI/4) {
        return RIGHT;
    }
    if(abs(self.getAngle()) >= 3*PI/4) {
        return LEFT;
    }

    if(self.getAngle() > 0) {
        return DOWN;
    }

    if(self.getAngle() < 0) {
        return UP;
    }
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

bool Inside(Point p, TileType tile, int width, int length) {
    int x = p.x_ % 800;
    int y = p.y_ % 800;
    Point new_point(x,y);
    if(tile == EMPTY) {
        return true;
    }
    if(GetDistance(new_point, Point(0,0)) < 80 + length || GetDistance(new_point, Point(800,0)) < 80 + length || GetDistance(new_point, Point(0,800)) < 80 + length || GetDistance(new_point, Point(800,800)) < 80 + length) {
        return true;
    }

    if(tile == LEFT_HEADED_T || tile == VERTICAL || tile == RIGHT_TOP_CORNER || tile == RIGHT_BOTTOM_CORNER) {
        if(new_point.x_ > 800 - 80 - width) {
            return true;
        }
    }

    if(tile == RIGHT_HEADED_T || tile == VERTICAL || tile == LEFT_TOP_CORNER || tile == LEFT_BOTTOM_CORNER) {
        if(new_point.x_ < 80 + width) {
            return true;
        }
    }

    if(tile == TOP_HEADED_T || tile == HORIZONTAL || tile == LEFT_BOTTOM_CORNER || tile == RIGHT_BOTTOM_CORNER) {
        if(new_point.y_ > 800 - 80 - width) {
            return true;
        }
    }

    if(tile == BOTTOM_HEADED_T || tile == HORIZONTAL || tile == LEFT_TOP_CORNER || tile == RIGHT_TOP_CORNER) {
        if(new_point.y_ < 80 + width) {
            return true;
        }
    }
    return false;

}

Point CurrentTile(Point self) {
    return CurrentTile(self.x_, self.y_);
}

template<class M>
Point CurrentTile(const M& self) {
    return CurrentTile(self.getX(), self.getY());
}

vector<Point> PathFromPreds(map<Edge, Edge> preds, Point end, Point start) {
    vector<Point> answer;

    auto tempStart = Edge(make_pair(end, Point(-1, -1)));

    for(; tempStart.second != start; tempStart = preds[tempStart]) {
        if(tempStart.second != Point(-1, -1)) {
            answer.push_back(tempStart.second);
        }
    }
    answer.push_back(start);
    reverse(answer.begin(), answer.end());
    Print(answer);
    return answer;
}

vector<Point> Dijkstra(EdgeBasedGraph graph, Point startPoint, Point endPoint, Direction direction) {
    if(direction == RIGHT) {
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_ + 1, startPoint.y_)))] = 0;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_ - 1, startPoint.y_)))] = 20;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_, startPoint.y_ + 1)))] = 2;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_, startPoint.y_ - 1)))] = 2;
    }

    if(direction == LEFT) {
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_ + 1, startPoint.y_)))] = 20;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_ - 1, startPoint.y_)))] = 0;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_, startPoint.y_ + 1)))] = 2;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_, startPoint.y_ - 1)))] = 2;
    }
    if(direction == UP) {
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_ + 1, startPoint.y_)))] = 2;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_ - 1, startPoint.y_)))] = 2;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_, startPoint.y_ + 1)))] = 20;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_, startPoint.y_ - 1)))] = 0;
    }
    if(direction == DOWN) {
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_ + 1, startPoint.y_)))] = 2;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_ - 1, startPoint.y_)))] = 2;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_, startPoint.y_ + 1)))] = 0;
        graph.edge_graph_edges_[make_pair(Edge(make_pair(Point(-1,-1), startPoint)), make_pair(startPoint, Point(startPoint.x_, startPoint.y_ - 1)))] = 20;
    }

    priority_queue<pair<int, Edge>> q;
    map<Edge, int> dist;
    for(auto e : graph.edge_graph_vertices_) {
        dist[e] = INFINITY;
    }

    map<Edge, Edge> pred;
    q.push(make_pair(0, make_pair(Point(-1,-1), startPoint)));
    dist[make_pair(Point(-1,-1), startPoint)] = 0;
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
            return LEFT;
        } else if (from.x_ == to.x_ - 1) {
            return RIGHT;
        }
    }

    if(from.x_ == to.x_) {
        if(from.y_ == to.y_ + 1) {
            return UP;
        } else if (from.y_ == to.y_ - 1) {
            return DOWN;
        }
    }
    return UP;
}

Point GetPointAfterRotation(Point center, Point p, double angle) {
    return Point((p.x_ - center.x_) * cos(angle) -  (p.y_ - center.y_) * sin(angle) + center.x_, (p.x_ - center.x_) * sin(angle) +  (p.y_ - center.y_) * cos(angle) + center.y_);
}


pair<double, double> GetPointAfterRotation(double x, double y, double angle) {
    return make_pair(x * cos(angle) -  y * sin(angle), x * sin(angle) +  y * cos(angle));
}

bool Intersects(Point p, double angle, const World& world) {
    Point p1 = GetPointAfterRotation(p, Point(p.x_ + 105, p.y_ + 70), angle);
    Point p2 = GetPointAfterRotation(p, Point(p.x_ + 105, p.y_ - 70), angle);
    Point p3 = GetPointAfterRotation(p, Point(p.x_ - 105, p.y_ + 70), angle);
    Point p4 = GetPointAfterRotation(p, Point(p.x_ - 105, p.y_ - 70), angle);
    Point p5 = GetPointAfterRotation(p, Point(p.x_, p.y_ + 70), angle);
    Point p6 = GetPointAfterRotation(p, Point(p.x_, p.y_ - 70), angle);
    //p1.Print();
    //p2.Print();
    //p3.Print();
    return Inside(p1, world.getTilesXY()[p1.x_ / 800][p1.y_ / 800], 0, 0) || Inside(p2, world.getTilesXY()[p2.x_ / 800][p2.y_ / 800], 0, 0) || Inside(p3, world.getTilesXY()[p3.x_ / 800][p3.y_ / 800], 0, 0) || Inside(p4, world.getTilesXY()[p4.x_/ 800][p4.y_/ 800], 0, 0)
           || Inside(p5, world.getTilesXY()[p5.x_/ 800][p5.y_/ 800], 0, 0) || Inside(p6, world.getTilesXY()[p6.x_/ 800][p6.y_/ 800], 0, 0);
}

bool Intersects(Point p, const World& world) {
    return Inside(p, world.getTilesXY()[p.x_ / 800][p.y_ / 800], 0, 0);
}

bool Intersects(const Car& self,  const World& world) {
    return Intersects(Point(self.getX(), self.getY()), self.getAngle(), world);
}
vector<Point> MyStrategy::bestPath(const Car& self, const World& world, const Game& game) {


    auto space = world.getTilesXY();
    Point startPoint = CurrentTile(self);
    Direction startDirection = world.getStartingDirection();
    graph = ConvertToEdgeBasedGraph(space);
    cout << "Graph build" << endl;
    AddStartAndEndNodes(graph, startPoint);
    cout << "Start and end nodes added" << endl;

    vector<Point> final_answer;
    for(int i = 0; i < world.getWaypoints().size(); ++i) {
        Point from = Point(world.getWaypoints()[i][0], world.getWaypoints()[i][1]);
        Point to = Point(world.getWaypoints()[(i+1) % world.getWaypoints().size()][0], world.getWaypoints()[(i+1) % world.getWaypoints().size()][1]);
        Direction direction = i == 0 ? world.getStartingDirection() : GetDirection(final_answer[final_answer.size() - 2], final_answer[final_answer.size() - 1]);
        vector<Point> answer = Dijkstra(graph, from, to, direction);
        cout << "Dijkstra run completed" << endl;
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

bool isTurn(int index, vector<Point>& points) {
    int n = points.size();
    return abs(points[(index + n - 1) % n].x_ - points[(index + 1) % n].x_) == 1 and  abs(points[(index + n - 1) % n].y_ - points[(index + 1) % n].y_) == 1;
}

bool LastFiveAroundZero(vector<double>& speedVector) {
    for(int i = max(0, (int)speedVector.size() - 5); i < speedVector.size(); ++i) {
        if(speedVector[i] > 0.1) {
            return false;
        }
    }
    return true;
}

bool isRightTurn(int index, vector<Point>& points) {
    if(!isTurn(index, points)) {
        return false;
    }
    index = (index + points.size()) % points.size();
    bool xIncr = points[(index + points.size() - 1) % points.size()].x_ < points[(index + 1) % points.size()].x_;
    bool yIncr = points[(index + points.size() - 1) % points.size()].y_ < points[(index + 1) % points.size()].y_;
    if (xIncr == yIncr) {
        if (points[index].x_ != points[(index + points.size() - 1) % points.size()].x_) {
            return true;
        } else {
            return false;
        }
    }
    if (xIncr != yIncr) {
        if (points[index].y_ != points[(index + points.size() - 1) % points.size()].y_) {
            return true;
        } else {
            return false;
        }
    }
}

bool isLeftTurn(int index, vector<Point>& points) {
    return isTurn(index, points) && !isRightTurn(index, points);
}

int lengthOfTheLine(int index, vector<Point>& points) {
    int answer = 2;



    int x_diff = points[index].x_ - points[(index + points.size() - 1) % points.size()].x_;
    int y_diff = points[index].y_ - points[(index + points.size() - 1) % points.size()].y_;
    int curr = index + 1;


    while(true) {
        Point temp(points[(curr + points.size() - 1) % points.size()].x_ + x_diff, points[(curr + points.size() - 1) % points.size()].y_ + y_diff );
        if(temp != points[curr % points.size()]) {
            break;
        }
        answer++;
        curr++;
        if(curr == points.size()){
            curr = 0;
        }
    }
    return answer;
}

bool IsBorder(int index, vector<Point>& points, int length, const World& world) {
    Point a = points[index];
    Point b = points[(index+1) % points.size()];
    Point borderPoint = points[(index + length - 2) % points.size()];

    Direction  dir = GetDirection(a,b);
    if(dir == UP) {
        return Inside(Point(400, 0), world.getTilesXY()[borderPoint.x_][borderPoint.y_], 0, 0);
    }
    if(dir == DOWN) {
        return Inside(Point(400, 799), world.getTilesXY()[borderPoint.x_][borderPoint.y_], 0, 0);
    }
    if(dir == LEFT) {
        return Inside(Point(0, 400), world.getTilesXY()[borderPoint.x_][borderPoint.y_], 0, 0);
    }
    if(dir == RIGHT) {
        return Inside(Point(799, 400), world.getTilesXY()[borderPoint.x_][borderPoint.y_], 0, 0);
    }

}

void SetCoordsInternal(int& xCoord, int& yCoord, vector<Point>& way, int curr_index, Move& move, int coordsize) {

    if(way[(curr_index + 1) % way.size()].x_ > way[curr_index].x_) {
        xCoord += coordsize;
    }
    if(way[(curr_index + 1) % way.size()].x_ < way[curr_index].x_) {
        xCoord -= coordsize;
    }

    if(way[(curr_index + way.size() - 1) % way.size()].x_ > way[curr_index].x_) {
        xCoord += coordsize;
    }
    if(way[(curr_index + way.size() - 1) % way.size()].x_ < way[curr_index].x_) {
        xCoord -= coordsize;
    }

    if(way[(curr_index + way.size() - 1) % way.size()].y_ < way[curr_index].y_) {
        yCoord -= coordsize;
    }
    if(way[(curr_index + way.size() - 1) % way.size()].y_ > way[curr_index].y_) {
        yCoord += coordsize;
    }

    if(way[(curr_index + 1) % way.size()].y_ < way[curr_index].y_) {
        yCoord -= coordsize;
    }
    if(way[(curr_index + 1) % way.size()].y_ > way[curr_index].y_) {
        yCoord += coordsize;
    }
}

void SetCoordsInternalLL(int& xCoord, int& yCoord, vector<Point>& way, int curr_index, Move& move, int coordsize) {

    if(way[(curr_index + 1) % way.size()].x_ > way[curr_index].x_) {
        xCoord -= coordsize;
    }
    if(way[(curr_index + 1) % way.size()].x_ < way[curr_index].x_) {
        xCoord += coordsize;
    }

/*    if(way[(curr_index + way.size() - 1) % way.size()].x_ > way[curr_index].x_) {
        xCoord += coordsize;
    }
    if(way[(curr_index + way.size() - 1) % way.size()].x_ < way[curr_index].x_) {
        xCoord -= coordsize;
    }*/

/*    if(way[(curr_index + way.size() - 1) % way.size()].y_ < way[curr_index].y_) {
        yCoord -= coordsize;
    }
    if(way[(curr_index + way.size() - 1) % way.size()].y_ > way[curr_index].y_) {
        yCoord += coordsize;
    }*/

    if(way[(curr_index + 1) % way.size()].y_ < way[curr_index].y_) {
        yCoord += coordsize;
    }
    if(way[(curr_index + 1) % way.size()].y_ > way[curr_index].y_) {
        yCoord -= coordsize;
    }
}


void SetCoordsInternal(int& xCoord, int& yCoord, vector<Point>& way, int curr_index, Move& move, int coordsize, int coordsize2) {
//OK
    if(way[(curr_index + 1) % way.size()].x_ > way[curr_index].x_ && way[(curr_index + 2) % way.size()].y_ > way[curr_index].y_ ) {
        xCoord += coordsize;
        yCoord -= coordsize2;
    }
//ok
    if(way[(curr_index + 1) % way.size()].x_ > way[curr_index].x_ && way[(curr_index + 2) % way.size()].y_ < way[curr_index].y_ ) {
        xCoord += coordsize;
        yCoord += coordsize2;
    }

//ok
    if(way[(curr_index + 1) % way.size()].x_ < way[curr_index].x_ && way[(curr_index + 2) % way.size()].y_ < way[curr_index].y_ ) {
        xCoord -= coordsize;
        yCoord += coordsize2;
    }

    //OK

    if(way[(curr_index + 1) % way.size()].x_ < way[curr_index].x_ && way[(curr_index + 2) % way.size()].y_ > way[curr_index].y_ ) {
        xCoord -= coordsize;
        yCoord -= coordsize2;
    }

//ok
    if(way[(curr_index + 1) % way.size()].y_ > way[curr_index].y_ && way[(curr_index + 2) % way.size()].x_ > way[curr_index].x_ ) {
        xCoord -= coordsize2;
        yCoord += coordsize;
    }
//OK
    if(way[(curr_index + 1) % way.size()].y_ > way[curr_index].y_ && way[(curr_index + 2) % way.size()].x_ < way[curr_index].x_ ) {
        xCoord += coordsize2;
        yCoord += coordsize;
    }


    if(way[(curr_index + 1) % way.size()].y_ < way[curr_index].y_ && way[(curr_index + 2) % way.size()].x_ > way[curr_index].x_ ) {
        xCoord -= coordsize2;
        yCoord -= coordsize;
    }

    if(way[(curr_index + 1) % way.size()].y_ < way[curr_index].y_ && way[(curr_index + 2) % way.size()].x_ < way[curr_index].x_ ) {
        xCoord += coordsize2;
        yCoord -= coordsize;
    }
}

void SetCoords(int& xCoord, int& yCoord, vector<Point>& way, int curr_index, Move& move) {


        if( isRightTurn(curr_index, way) and isRightTurn(curr_index + 1, way)) {
            xCoord = way[curr_index].x_ * 800 + 400 ;
            yCoord = way[curr_index].y_ * 800 + 400 ;
            SetCoordsInternalLL(xCoord, yCoord, way, curr_index, move, -200);
            return;
        }
        if( isLeftTurn(curr_index, way) and isLeftTurn(curr_index + 1, way)) {
            xCoord = way[curr_index].x_ * 800 + 400 ;
            yCoord = way[curr_index].y_ * 800 + 400 ;
            SetCoordsInternalLL(xCoord, yCoord, way, curr_index, move, -200);
            return;
        }

        if((isLeftTurn(curr_index, way) and isRightTurn(curr_index + 1, way)) || (isRightTurn(curr_index, way) and isLeftTurn(curr_index + 1, way))) {
            xCoord = way[curr_index].x_ * 800 + 400 ;
            yCoord = way[curr_index].y_ * 800 + 400 ;
            SetCoordsInternal(xCoord, yCoord, way, curr_index, move, 310, 90);
            return;
        }

//


    if(isTurn(curr_index, way)) {
        xCoord = way[curr_index].x_ * 800 + 400 ;
        yCoord = way[curr_index].y_ * 800 + 400 ;
        SetCoordsInternal(xCoord, yCoord, way, curr_index, move, 229);
        return;
    }

    int l = lengthOfTheLine(curr_index, way);

    if(isLeftTurn((curr_index + l - 1) % way.size(), way) and isLeftTurn((curr_index + l - 2) % way.size(), way)) {
        Direction d = GetDirection(way[curr_index], way[(curr_index + 1) % way.size()]);
        if(d == UP) {
            if(xCoord % 800 < 400)
                xCoord += 150;
        }
        if(d == DOWN) {
            if(xCoord % 800 > 400)
                xCoord -= 150;
        }
        if(d == LEFT) {
            if(yCoord % 800 > 400)
                yCoord -= 150;
        }
        if(d == RIGHT) {
            if(yCoord % 800 < 400)
                yCoord += 150;
        }

        return;
    }

    if(isRightTurn((curr_index + l - 1) % way.size(), way) and isRightTurn((curr_index + l - 2) % way.size(), way)) {
        Direction d = GetDirection(way[curr_index], way[(curr_index + 1) % way.size()]);
        if(d == UP) {
            if(xCoord % 800 > 400)
                xCoord -= 110;
        }
        if(d == DOWN) {
            if(xCoord % 800 < 400)
                xCoord += 110;
        }
        if(d == LEFT) {
            if(yCoord % 800 < 400)
            yCoord += 110;
        }
        if(d == RIGHT) {
            if(yCoord % 800 > 400)
            yCoord -= 110;
        }

        return;
    }

}


void FireInTheHole(const Car& self, const World& world, Move& move) {
    for(auto car : world.getCars()) {
        if(car.getId() == self.getId()) {
            continue;
        }
        if(car.getDurability() == 0 || car.getDurability() > 45 * self.getProjectileCount()) {
            continue;
        }
        double xCoord = car.getX();
        double yCoord = car.getY();
        double xSpeed = car.getSpeedX();
        double ySpeed = car.getSpeedY();
        double projectileSpeed = 60.0;
        double angle = self.getAngle();
        double xProjectSpeed = projectileSpeed * cos(angle);
        double yProjectSpeed = projectileSpeed * sin(angle);
        if(self.getType() == BUGGY) {
            for(int time = 0; time < 50; ++time) {
                if(abs((xCoord + (double)time * xSpeed) - (self.getX() + (double)time * xProjectSpeed)) < 70.0 and abs((yCoord + (double)time * ySpeed) - (self.getY() + (double)time * yProjectSpeed)) < 70.0 ) {
                    move.setThrowProjectile(true);
                }
            }
        } else {
            for(int time = 0; time < 50; ++time) {
                if(Intersects(Point(self.getX() + (double)time * xProjectSpeed, self.getY() + (double)time * yProjectSpeed), world)) {
                    break;
                }
                if(abs((xCoord + (double)time * xSpeed) - (self.getX() + (double)time * xProjectSpeed)) < 70.0 and abs((yCoord + (double)time * ySpeed) - (self.getY() + (double)time * yProjectSpeed)) < 70.0 ) {
                    move.setThrowProjectile(true);
                }
            }
        }

    }
}




void ChangeCoordsToBonus(int& xCoord, int& yCoord, const World& world, const Car& self, Point next2, Point next3) {
    Point next = CurrentTile(xCoord, yCoord);
    int min_distance = 10000;
    if(world.getMapName() == "default" || world.getMapName() == "map01" || world.getMapName() == "map02" ) {
        return;
    }
    for(auto bonus : world.getBonuses()) {
        if(((CurrentTile(bonus) == next) || (CurrentTile(bonus) == next2) || (CurrentTile(bonus) == next3)) && abs(self.getAngleTo(bonus)) < PI/8.0) {
            if(min_distance > bonus.getDistanceTo(self)) {
                min_distance = bonus.getDistanceTo(self);
                xCoord = bonus.getX();
                yCoord = bonus.getY();
            }
        }
    }
}



bool InsideDoubleTurn(int index, vector<Point>& way) {
    return (isLeftTurn(index - 1, way) and isLeftTurn(index, way)) || (isRightTurn(index - 1, way) and isRightTurn(index, way)) ||
           (isLeftTurn(index - 2, way) and isLeftTurn(index - 1, way)) || (isRightTurn(index - 2, way) and isRightTurn(index - 1, way)) ;
}



bool NearToCenterOfDoubleTurn(const Car& self, int index, vector<Point>& way) {
    int start_of_turn = 0;
    if((isLeftTurn(index - 1, way) and isLeftTurn(index, way)) || (isRightTurn(index - 1, way) and isRightTurn(index, way))) {
        start_of_turn = index - 1;
    }

    if((isLeftTurn(index - 2, way) and isLeftTurn(index - 1, way)) || (isRightTurn(index - 2, way) and isRightTurn(index - 1, way))) {
        start_of_turn = index - 2;
    }

    Direction dir = GetDirection(way[(start_of_turn + way.size()) % way.size()], way[(start_of_turn + 1 + way.size()) % way.size()]);
    if(dir == LEFT) {
        if(abs(self.getAngle()) > 2.4) {
            return true;
        }
    }
    if(dir == RIGHT) {
        if(abs(self.getAngle()) < 0.85) {
            return true;
        }
    }
    if(dir == DOWN) {
        if(self.getAngle() < PI/2.0 + 0.40 && self.getAngle() > PI/2.0 - 0.40 ) {
            return true;
        }
    }
    if(dir == UP) {
        if(self.getAngle() < -PI/2.0 + 0.40 && self.getAngle() > -PI/2.0 - 0.40 ) {
            return true;
        }
    }
    return false;
}

Point getNextPoint(Point cur, double speedX, double speedY, const Car& self) {
    return Point(cur.x_ + speedX, cur.y_ + speedY);
}

inline double limit(double val, double lim)
{
    return max(-lim, min(lim, val));
}

Point getNextPoint(Point cur, double speedX, double speedY, const Car& self, const Game& game, double angle) {

    auto carType = self.getType();
    double updateFactor = 0.1;
    double accelerationFactor = self.getType() == BUGGY ? game.getBuggyEngineForwardPower() : game.getJeepEngineForwardPower();
    accelerationFactor /= self.getMass();
    double accelerationX = accelerationFactor * self.getEnginePower() * cos(self.getAngle());
    double accelerationY = accelerationFactor * self.getEnginePower() * sin(self.getAngle());
    double speed = GetSpeed(self);
    double angSpeed = self.getAngularSpeed();
    double baseAngeularSpeed = game.getCarAngularSpeedFactor() * self.getWheelTurn() * (speedX * cos(angle) + speedY * sin(angle));

    double lenghtwiseVelocityChange = game.getCarLengthwiseMovementFrictionFactor() * updateFactor;
    double crosswiseVelocityChange = game.getCarCrosswiseMovementFrictionFactor() * updateFactor;


    double frictMult = pow(1 - game.getCarRotationAirFrictionFactor(), 0.1);

    double newX = cur.x_;
    double newY = cur.y_;


    for(int i = 0; i < 10; ++i) {
          newX += speedX*updateFactor;
          newY += speedY*updateFactor;
              speedX += accelerationX;
              speedY += accelerationY;
              speedX *= game.getCarMovementAirFrictionFactor();
              speedY *= game.getCarMovementAirFrictionFactor();
              double temp = speedX*cos(angle) + speedY*sin(angle);
              speedX -= limit(speedX * cos(angle), lenghtwiseVelocityChange) * speedX + limit(temp, crosswiseVelocityChange)*GetPointAfterRotation(cos(angle), sin(angle), PI/2).first;
              speedY -= limit(speedY * sin(angle), lenghtwiseVelocityChange) * speedY + limit(temp, crosswiseVelocityChange)*GetPointAfterRotation(cos(angle), sin(angle), PI/2).second;
              angle += angSpeed*0.1;
              angSpeed = baseAngeularSpeed + (angSpeed - baseAngeularSpeed) * frictMult;
    }
    return Point(newX, newY);
}

bool OutInTenTicks(const Car& self, const World& world, double angle, vector<Point>& way, int index, int n = 10) {

    Point cur(self.getX(), self.getY());
    for(int i = 0; i < n; ++i) {
        Point next = getNextPoint(cur, self.getSpeedX(), self.getSpeedY(), self);
        cur = next;
    }
    if(CurrentTile(cur) == CurrentTile(self)) {
        return false;
    }

    Direction dir = GetDirection(way[(index + way.size() - 1) % way.size()], way[index]);
    Direction dir2 = GetDirection(CurrentTile(self), CurrentTile(cur));
    return dir != dir2;
}

void DifferentThingsWithLengthOfTheLine(const Car& self, const World& world, const Game& game, Move& move, int curr_index, vector<Point>& way, int& xCoord, int& yCoord, bool isStunned) {
    int length = lengthOfTheLine(curr_index, way);
    if(length > 3 ) {
        if (self.getRemainingNitroTicks() == 0) {
               ChangeCoordsToBonus(xCoord, yCoord, world, self, way[(curr_index + 1) % way.size()], way[(curr_index + 2) % way.size()]);
        }
    }

    if(length >= 5 && IsBorder(curr_index, way, length, world)) {
        //TODO: something about angle (should be close to pi*n/2)
        if(abs(self.getAngleTo(xCoord, yCoord)) < 0.1) {
            int weapons = 0;
            for(auto car : world.getCars()) {
                if (car.getId() == self.getId()) {
                    continue;
                }

                if(self.getDistanceTo(car) < 600) {
                    weapons += car.getProjectileCount() > 0 ? 1 : 0;
                }
            }
            if(weapons < 2) {
                move.setUseNitro(true);
            }
        }
    }

    if((length <= 3 and GetSpeed(self) > 30) || (length <= 2 and GetSpeed(self) > 20)) {
        //TODO: something about angle (should be close to pi*n/2)
        //move.setUseNitro(true);
        if(!((isLeftTurn(curr_index, way) and isRightTurn(curr_index + 1, way)) || (isRightTurn(curr_index, way) and isLeftTurn(curr_index + 1, way)))) {
            move.setBrake(true);
        }

        if(((isLeftTurn(curr_index, way) and isRightTurn(curr_index + 1, way)) || (isRightTurn(curr_index, way) and isLeftTurn(curr_index + 1, way)))) {
            if(!isTurn(curr_index - 1, way) and GetSpeed(self) > 20)
                move.setBrake(true);
        }
    }

    if(InsideDoubleTurn(curr_index, way)) {
        if(GetSpeed(self) > 10 && abs(self.getWheelTurn()) > 0.9) {
            if(NearToCenterOfDoubleTurn(self, curr_index, way)) {
                move.setBrake(true);
            }
        }
    }

    //if(!isStunned && GetSpeed(self) > 5 && OutInTenTicks(self, world, self.getAngle(), way, curr_index)) {
    //    move.setBrake(true);
    //}

}



bool inSimpleTurn(vector<Point>& way, int index) {
    return isTurn(index, way) && !((isLeftTurn(index, way) and isRightTurn(index + 1, way)) || (isRightTurn(index, way) and isLeftTurn(index + 1, way)));
}

void SpillOil(const Car& self, const World& world, Move& move) {
    for(auto car : world.getCars()) {
        if (car.getId() == self.getId()) {
            continue;
        }

        if(abs(self.getAngleTo(car)) > 3.5*PI/4 && abs(car.getAngleTo(self)) <= PI/6 && self.getDistanceTo(car) < 2000) {
            move.setSpillOil(true);
        }
    }
}



double getWheelTurn(double turn, const Game& game, double angle, double speedX, double speedY) {
    double speed = speedX*speedX + speedY*speedY;
    if(angle < 0) {
        double sumangle = 0;
        for(double i = turn; i < 0; i += 0.05) {
            sumangle += (speedX * cos(angle + sumangle) + speedY * sin(angle + sumangle)) * game.getCarAngularSpeedFactor() * (i);
            speedX = cos(angle + sumangle) * speed;
            speedY = sin(angle + sumangle) * speed;
            //cout << "sumangle - " << sumangle << endl;
        }


        //cout << "sumangle - " << sumangle << endl;
        if(sumangle > angle)
            return turn;
        //cout << "Hi!" << endl;
        return turn + 0.05;
    }
    else {
        double sumangle = 0;
        for(double i = turn; i > 0; i -= 0.05) {
            sumangle += (speedX * cos(angle + sumangle) + speedY * sin(angle + sumangle)) * game.getCarAngularSpeedFactor() * (i);
            speedX = cos(angle + sumangle) * speed;
            speedY = sin(angle + sumangle) * speed;
            //cout << "sumangle - " << sumangle << endl;
        }
        //cout << "sumangle - " << sumangle << endl;
        if(sumangle < angle)
            return turn;
        //cout << "Not Hi!" << endl;
        return turn - 0.05;
    }

}


double getWheelTurn(const Car& self, const Game& game, double angle) {
    double speed = GetSpeed(self);
    double speedX = cos(speed);
    double speedY = sin(speed);

    if(angle < 0) {
            double sumangle = 0;
            double turn = self.getWheelTurn();
            for(double i = turn; i < 0; i += 0.05) {
                sumangle += (speedX * cos(angle + sumangle) + speedY * sin(angle + sumangle)) * game.getCarAngularSpeedFactor() * (i);
                speedX = cos(angle + sumangle) * speed;
                speedY = sin(angle + sumangle) * speed;
          //      cout << "sumangle - " << sumangle << endl;
            }
            //cout << "sumangle - " << sumangle << endl;
            if(sumangle > angle)
                return -1;
            //cout << "Hi!" << endl;
            return turn + 0.05;
    }
    else {
            double turn = self.getWheelTurn();
            double sumangle = 0;
            for(double i = turn; i > 0; i -= 0.05) {
                sumangle += (speedX * cos(angle + sumangle) + speedY * sin(angle + sumangle)) * game.getCarAngularSpeedFactor() * (i);
                speedX = cos(angle + sumangle) * speed;
                speedY = sin(angle + sumangle) * speed;
        //        cout << "sumangle - " << sumangle << endl;
            }
            //cout << "sumangle - " << sumangle << endl;
            if(sumangle < angle)
                return 1;
            //cout << "Not Hi!" << endl;
            return turn - 0.05;
    }
}


double GetAngle(double a, double b) {
    double c = b - a;
    if(c > PI) {
        c -= 2*PI;
    } else if(c < -PI) {
        c += 2*PI;
    }
    return c;

}



int SetWheelSubroutine(double currentWheelTurn, double angle, int currentX, int currentY, int desiredX, int desiredY, double desiredAngle, const Car& self, const Game& game, double max, double& dist, const World& world) {
    double myAngle = self.getAngle();
    int ticks = 0;
    double speedX = self.getSpeedX();
    double speedY = self.getSpeedY();
    while(true) {
        if((max < 0 && currentWheelTurn < max) || (max > 0 && currentWheelTurn > max))
            break;
        currentWheelTurn += angle > 0 ? 0.05 : -0.05;
        //cout << "Current whell turn - " << currentWheelTurn << endl;
        Point new_point = getNextPoint(Point(currentX, currentY), speedX, speedY, self, game, myAngle);
        if(Intersects(new_point, myAngle, world)) {
            dist = 10000;
            return 10000;
        }
        //new_point.Print();
        currentX = new_point.x_;
        currentY = new_point.y_;
        double angle_add = (speedX * cos(myAngle) + speedY * sin(myAngle)) * game.getCarAngularSpeedFactor() * currentWheelTurn;
        //cout << "Add angle - " << angle_add<< endl;

        myAngle += angle_add;
        //cout << "New angle - " << myAngle << endl;

        speedX = GetSpeed(self) * cos(myAngle);
        //cout << "New sppedX - " << speedX<< endl;

        speedY = GetSpeed(self) * sin(myAngle);
        //cout << "New sppedY - " << speedY<< endl;
        if(ticks > 500) {
            dist = 1000000;
            return 10000;
        }

        ticks++;
    }

    while(getWheelTurn(currentWheelTurn, game, GetAngle(myAngle, desiredAngle), speedX, speedY) == currentWheelTurn) {
        Point new_point = getNextPoint(Point(currentX, currentY), speedX, speedY, self, game, myAngle);
        if(Intersects(new_point, myAngle, world)) {
            //cout << "Inside ";
            //new_point.Print();
            dist = 1000000;
            return 10000;
        }

        //new_point.Print();
        currentX = new_point.x_;
        currentY = new_point.y_;
        double angle_add = (speedX * cos(myAngle) + speedY * sin(myAngle)) * game.getCarAngularSpeedFactor() * currentWheelTurn;
       // cout << "Add angle - " << angle_add<< endl;

        myAngle += angle_add;
       // cout << "New angle - " << myAngle << endl;

        speedX = GetSpeed(self) * cos(myAngle);
       // cout << "New sppedX - " << speedX<< endl;

        speedY = GetSpeed(self) * sin(myAngle);
       // cout << "New sppedY - " << speedY<< endl;/
        ticks++;
        if(ticks > 500) {
            dist = 1000000;
            return 10000;
        }
        if(GetDistance(Point(currentX, currentY), Point(desiredX, desiredY)) > 3000){
            dist = 1000000;
            return 1000000000;
        }
    }

    while(abs(currentWheelTurn) > 0.05) {
        currentWheelTurn -= currentWheelTurn > 0 ? 0.05 : -0.05;
        Point new_point = getNextPoint(Point(currentX, currentY), speedX, speedY, self, game, myAngle);
        if(Intersects(new_point, myAngle, world)) {
            dist = 1000000;
            return 10000;
        }

        //new_point.Print();
        currentX = new_point.x_;
        currentY = new_point.y_;
        double angle_add = (speedX * cos(myAngle) + speedY * sin(myAngle)) * game.getCarAngularSpeedFactor() * currentWheelTurn;
       // cout << "Add angle - " << angle_add<< endl;

        myAngle += angle_add;
       // cout << "New angle - " << myAngle<< endl;

        speedX = GetSpeed(self) * cos(myAngle);
       // cout << "New sppedX - " << speedX<< endl;

        speedY = GetSpeed(self) * sin(myAngle);
       // cout << "New sppedY - " << speedY<< endl;
        if(ticks > 500) {
            dist = 1000000;
            return 10000;
        }
        ticks++;
    }
    dist = abs(desiredAngle) == PI/2 ? abs(currentX - desiredX) : abs(currentY - desiredY);
    return ticks;
}



bool SetWheelTurnInSimpleTurn(const Car& self, const Game& game, Move& move, int curr_ind, vector<Point>& way, const World& world) {
    double myAngle = self.getAngle();
    double desiredAngle =  0.0;
    int currentX = self.getX();
    int currentY = self.getY();

    int desiredX = inSimpleTurn(way, curr_ind) ? way[(curr_ind + 1) % way.size()].x_ * 800 + 400 : way[(curr_ind) % way.size()].x_ * 800 + 400;
    int desiredY = inSimpleTurn(way, curr_ind) ? way[(curr_ind + 1) % way.size()].y_ * 800 + 400 : way[(curr_ind) % way.size()].y_ * 800 + 400;
    //cout << "desired - " << desiredX << ", " << desiredY << endl;
    double currentWheelTurn = self.getWheelTurn();
    Direction  finaldir = GetDirection(way[curr_ind], way[(curr_ind + 1) % way.size()]);

    if(finaldir == UP) {
        cout << "UP" << endl;
        desiredAngle = -PI/2;
    }
    if(finaldir == DOWN) {
        cout << "DOWN" << endl;
        desiredAngle = PI/2;
    }

    if(finaldir == LEFT) {
        cout << "LEFT" << endl;

       desiredAngle = PI;
    }
    if(finaldir == RIGHT) {
        cout << "RIGHT" << endl;

        desiredAngle = 0.0;
    }
    double distance = 0;
    int min_ticks = 10000;
    double angle = GetAngle(myAngle, desiredAngle);


    bool set = false;
    for(double i = 0.05; i <= 1.0; i += 0.05) {
        int ticks = SetWheelSubroutine(currentWheelTurn, angle, currentX, currentY, desiredX, desiredY, desiredAngle, self, game, angle > 0 ? i : -i, distance, world);
        //cout << "for i = " << i << ", distance found - " << distance << " - ticks -"<< ticks << endl;
        if(distance < 300.0) {
            if(ticks < min_ticks) {
                set = true;
                min_ticks = ticks;
                move.setWheelTurn(angle > 0 ? i : -i);
            }
        }
    }
    return set;
}

void MyStrategy::SetInitialCoords(int& xCoord, int& yCoord, const Car& self) {
    Direction dir;
    if(lostWay && additional_index == 0) {
        dir = GetSomeDirection(self);
    } else {
        dir = GetDirection(lostWay ? additional_way[additional_index - 1] : way[(curr_index + way.size() - 1) % way.size()], lostWay ? additional_way[additional_index] : way[(curr_index)]);
    }

    if(dir == UP || dir == DOWN) {

        xCoord = self.getX();
        if(xCoord % 800 < 250) {
            xCoord += 250 - (xCoord % 800);
        }
        if(xCoord % 800 > 550) {
            xCoord -= ((xCoord % 800) - 550 );
        }
    }
    if(dir == LEFT || dir == RIGHT) {
        yCoord = self.getY();
        if(yCoord % 800 < 250) {
            yCoord += 250 - (yCoord % 800);
        }

        if(yCoord % 800 > 550) {
            yCoord -= ((yCoord % 800) - 550 );
        }

    }

}





bool inSimpleOrDoubleTurn(int index, vector<Point>& way) {
    return inSimpleTurn(way, index) || (isLeftTurn(index, way) and isLeftTurn(index + 1, way)) || (isRightTurn(index, way) and isRightTurn(index + 1, way));
}

void MyStrategy::move(const Car& self, const World& world, const Game& game, Move& move) {
    if(self.isFinishedTrack()) {
        return;
    }

    if(way.size() == 0) {
        cout << "Way size is zero" << endl;
        way = bestPath(self, world, game);
    }


    if(curr_index == way.size()) {
        curr_index = 0;
    }

    if(int(world.getTick()) > removeStun) {
        isStunned = false;
    }

    Point current_tile = CurrentTile(self);

//    cout << "Current index is ";
    //cout << curr_index << endl;
//    cout << "Current tile is ";
    //current_tile.Print();
    //cout << "Current xy - " << self.getX() << " " << self.getY() << endl;

    if(curr_index != way.size() and current_tile == way[curr_index]) {
        curr_index++;
        additional_way.clear();
        lostWay = false;
    } else if(lostWay and additional_way[additional_index] == current_tile) {
        additional_index++;
    }

    if((!lostWay and curr_index > 0 && current_tile != way[curr_index - 1]) || (lostWay and current_tile != additional_way[(additional_index + additional_way.size() - 1) % additional_way.size()] )) {
        Point next(self.getNextWaypointX(), self.getNextWaypointY());
        for(int i = curr_index; ;++i) {
            if(i == way.size()) {
                i = 0;
            }
            if(next == way[i]) {
                curr_index = i;
                break;
            }
        }
        additional_index = 0;
        lostWay = true;
        additional_way = Dijkstra(graph, current_tile, next, GetSomeDirection(self));
        additional_way.push_back(way[(curr_index + 1) % way.size()]);
        additional_way.push_back(way[(curr_index + 2) % way.size()]);
        additional_way.push_back(way[(curr_index + 3) % way.size()]);
        additional_index++;
    }
    int xCoord = lostWay ? additional_way[additional_index].x_ * 800 + 0.5 * game.getTrackTileSize() : way[curr_index].x_ * 800 + 0.5 * game.getTrackTileSize();
    int yCoord = lostWay ? additional_way[additional_index].y_ * 800 + 0.5 * game.getTrackTileSize() : way[curr_index].y_ * 800 + 0.5 * game.getTrackTileSize();
    //if(!isTurn(lostWay ? additional_index : curr_index, lostWay ? additional_way : way))
    SetInitialCoords(xCoord, yCoord, self);
    if(lostWay) {
        SetCoords(xCoord, yCoord, additional_way, additional_index, move);
    } else {
        SetCoords(xCoord, yCoord, way, curr_index, move);
    }

    double angle = self.getAngleTo(xCoord, yCoord);
    double angleCar = self.getAngle();
    double engPower = self.getEnginePower();
    //cout << xCoord << " " << yCoord << " " << angle << " " << GetSpeed(self) << " " << engPower << endl;
    move.setEnginePower(1.0);

    FireInTheHole(self, world, move);

    speedVector.push_back(GetSpeed(self));

    if (world.getTick() > game.getInitialFreezeDurationTicks()) {

        DifferentThingsWithLengthOfTheLine(self, world, game, move, lostWay ? additional_index : curr_index, lostWay ? additional_way : way, xCoord, yCoord, isStunned);

        angle = self.getAngleTo(xCoord, yCoord);
        bool set = false;
        if(!isStunned) {
            if(isgreater(engPower, 0.0)) {
                if(inSimpleOrDoubleTurn(lostWay ? additional_index : curr_index, lostWay ? additional_way : way)) {
                    set = SetWheelTurnInSimpleTurn(self, game, move, lostWay ? additional_index : curr_index, lostWay ? additional_way : way, world);
                    if(!set) {
//                            if(abs(angle) < 1) {
//                                move.setWheelTurn(0);
//                            } else {
                            move.setWheelTurn(getWheelTurn(self, game, angle));
//                        }
                    }
                } else {
                    if(needLeft(angle)) {
                        double wt = getWheelTurn(self, game, angle);
                        move.setWheelTurn(wt);

                    } else {
                        double wt = getWheelTurn(self, game, angle);
                        move.setWheelTurn(wt);
                    }
                }
            } else {
                if(needLeft(angle)) {
                    move.setWheelTurn(1.0);
                } else {
                    move.setWheelTurn(-1.0);
                }
            }


            if(abs(self.getAngularSpeed()) > 0.02 && GetSpeed(self) > 15.0 && !set && lengthOfTheLine(lostWay ? additional_index - 1 : curr_index - 1, lostWay ? additional_way : way) <= 2) {
                    move.setBrake(true);
            }

            if(std::abs(angle) > 1.0 && GetSpeed(self) > 8) {
//                move.setBrake(true);
            }
            if(set) {
                move.setBrake(false);
            }
        } else {
            if(isless(engPower, 0.0)) {
                if(needLeft(angle)) {
                    move.setWheelTurn(1.0);
                } else {
                    move.setWheelTurn(-1.0);
                }
            } else {
                if(needLeft(angle)) {
                    move.setWheelTurn(-1.0);
                } else {
                    move.setWheelTurn(1.0);
                }
            }
            move.setEnginePower(-1.0);
        }

    }
    if(isgreater(engPower, 0.0)) {
        SpillOil(self, world, move);
    }
    if (world.getTick() > game.getInitialFreezeDurationTicks() + 10) {
        if(LastFiveAroundZero(speedVector) && removeStun + 180 < world.getTick()) {
            removeStun = world.getTick() + 80;
            isStunned = true;
        }
    }

    //cout << self.getWheelTurn() << " - wheel" << endl;
    cout << world.getTick() << " - tick" << endl;
}

MyStrategy::MyStrategy() {
    curr_index = 0;
    additional_index = 0;
    lostWay = false;
    isStunned = false;
    removeStun = -1;
}
