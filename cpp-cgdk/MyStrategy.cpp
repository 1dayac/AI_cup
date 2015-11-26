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
    return (abs(e1.first.x_ - e2.second.x_) == 0 && abs(e1.first.y_ - e2.second.y_) == 2) ||
           (abs(e1.first.x_ - e2.second.x_) == 2 && abs(e1.first.y_ - e2.second.y_) == 0);
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
                    edge_graph_edges[make_pair(e,e2)] = 3;
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
        return UP;
    }

    if(self.getAngle() < 0) {
        return DOWN;
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

Point CurrentTile(const Car& self) {
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
        dist[e] = 100000;
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

        //add all neighbours && end points
        Edge end = make_pair(target.second, Point(-1,-1));
        if(graph.edge_graph_vertices_.count(end) > 0 && dist[target] + graph.edge_graph_edges_[make_pair(target, end)] < dist[end] ) {
            dist[end] = dist[target] + graph.edge_graph_edges_[make_pair(target, end)];
            pred[end] = target;
            q.push(make_pair(-dist[end], end));
        }

        Edge left = make_pair(target.second, Point(target.second.x_ - 1,target.second.y_));

        if(graph.edge_graph_vertices_.count(left) > 0 && dist[target] + graph.edge_graph_edges_[make_pair(target, left)] < dist[left] ) {
            dist[left] = dist[target] + graph.edge_graph_edges_[make_pair(target, left)];
            pred[left] = target;
            q.push(make_pair(-dist[left], left));
        }

        Edge right = make_pair(target.second, Point(target.second.x_ + 1,target.second.y_));

        if(graph.edge_graph_vertices_.count(right) > 0 && dist[target] + graph.edge_graph_edges_[make_pair(target, right)] < dist[right] ) {
            dist[right] = dist[target] + graph.edge_graph_edges_[make_pair(target, right)];
            pred[right] = target;
            q.push(make_pair(-dist[right], right));
        }

        Edge up = make_pair(target.second, Point(target.second.x_,target.second.y_ - 1));

        if(graph.edge_graph_vertices_.count(up) > 0 && dist[target] + graph.edge_graph_edges_[make_pair(target, up)] < dist[up] ) {
            dist[up] = dist[target] + graph.edge_graph_edges_[make_pair(target, up)];
            pred[up] = target;
            q.push(make_pair(-dist[up], up));
        }

        Edge down = make_pair(target.second, Point(target.second.x_,target.second.y_ + 1));

        if(graph.edge_graph_vertices_.count(down) > 0 && dist[target] + graph.edge_graph_edges_[make_pair(target, down)] < dist[down] ) {
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


vector<Point> MyStrategy::bestPath(const Car& self, const World& world, const Game& game) {
    for(auto e : world.getWaypoints()) {
        cout << e[0] << " " << e[1] << endl;
    }
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
        for(Point elem : answer) {
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

bool isTurn(size_t index, vector<Point>& points) {
    size_t n = points.size();
    return points[(index - 1) % n].x_ != points[(index + 1) % n].x_ &&     points[(index - 1) % n].y_ != points[(index + 1) % n].y_;
}

bool LastFiveAroundZero(vector<double>& speedVector) {
    for(int i = max(0, (int)speedVector.size() - 5); i < speedVector.size(); ++i) {
        if(speedVector[i] > 0.1) {
            return false;
        }
    }
    return true;
}

bool isRightTurn(size_t index, vector<Point>& points) {
    if (index <= 0 || index >= points.size() - 1)
        return false;
    bool xIncr = points[(index - 1) % points.size()].x_ < points[(index + 1) % points.size()].x_;
    bool yIncr = points[(index - 1) % points.size()].y_ < points[(index + 1) % points.size()].y_;
    if (xIncr == yIncr) {
        if (points[index].x_ != points[(index - 1) % points.size()].x_) {
            return true;
        } else {
            return false;
        }
    }
    if (xIncr != yIncr) {
        if (points[index].y_ != points[(index - 1) % points.size()].y_) {
            return true;
        } else {
            return false;
        }
    }
}

bool isLeftTurn(size_t index, vector<Point>& points) {
    return !isRightTurn(index, points);
}

int lengthOfTheLine(size_t index, vector<Point>& points) {
    if(index <= 1) {
        return 0;
    }
    int answer = 2;
    int x_diff = points[index].x_ - points[index - 1].x_;
    int y_diff = points[index].y_ - points[index - 1].y_;
    int curr = index + 1;
    while(curr < points.size()) {
        Point temp(points[curr - 1].x_ + x_diff, points[curr - 1].y_ + y_diff );
        if(temp != points[curr]) {
            break;
        }
        answer++;
        curr++;
    }
    return answer;
}

void SetCoordsInternal(int& xCoord, int& yCoord, vector<Point>& way, size_t curr_index, Move& move, int coordsize) {

    if(way[(curr_index + 1) % way.size()].x_ > way[curr_index].x_) {
        xCoord += coordsize;
    }
    if(way[(curr_index + 1) % way.size()].x_ < way[curr_index].x_) {
        xCoord -= coordsize;
    }

    if(way[(curr_index - 1) % way.size()].x_ > way[curr_index].x_) {
        xCoord += coordsize;
    }
    if(way[(curr_index - 1) % way.size()].x_ < way[curr_index].x_) {
        xCoord -= coordsize;
    }

    if(way[(curr_index - 1) % way.size()].y_ < way[curr_index].y_) {
        yCoord -= coordsize;
    }
    if(way[(curr_index - 1) % way.size()].y_ > way[curr_index].y_) {
        yCoord += coordsize;
    }

    if(way[(curr_index + 1) % way.size()].y_ < way[curr_index].y_) {
        yCoord -= coordsize;
    }
    if(way[(curr_index + 1) % way.size()].y_ > way[curr_index].y_) {
        yCoord += coordsize;
    }
}

void SetCoords(int& xCoord, int& yCoord, vector<Point>& way, size_t curr_index, Move& move) {


    if(isTurn(curr_index, way) && isTurn(curr_index + 1, way)) {
        if( isRightTurn(curr_index, way) && isRightTurn(curr_index + 1, way)) {
            SetCoordsInternal(xCoord, yCoord, way, curr_index, move, 200);
            move.setSpillOil(true);
            return;
        }
        if( isLeftTurn(curr_index, way) && isLeftTurn(curr_index + 1, way)) {
            SetCoordsInternal(xCoord, yCoord, way, curr_index, move, 200);
            move.setSpillOil(true);
            return;
        }

    }

    if(isTurn(curr_index, way)) {
        SetCoordsInternal(xCoord, yCoord, way, curr_index, move, 269);
        move.setSpillOil(true);
        return;
    }
}

double MyStrategy::getWheelTurnW(double alpha, const Car& self, const Game& game){
	double turn=0;
	double turn_max = 1;
	double turn_min = -1;
	double dw_max = min(self.getWheelTurn() + 0.5, 1.0)*game.getCarAngularSpeedFactor() * pow((pow(self.getSpeedX(), 2) + pow(self.getSpeedY(), 2)), 0.5);
	double dw_min = max(self.getWheelTurn() - 0.5, -1.0)*game.getCarAngularSpeedFactor() * pow((pow(self.getSpeedX(), 2) + pow(self.getSpeedY(), 2)), 0.5);
	cout << "dw_min - " << dw_min << ", dw_max - " << dw_max << endl;
	cout<<"alpha=="<<alpha<<endl;
	//if (alpha == 0){ return 0; }
	if ((alpha <= self.getAngularSpeed() + dw_max) && (alpha >= self.getAngularSpeed() + dw_min)){
		cout << "return " << (alpha - self.getAngularSpeed() - dw_min) / (dw_max - dw_min) << endl;
		return (alpha - self.getAngularSpeed() - dw_min) / (dw_max - dw_min);
    }
    else{
		if (alpha > self.getAngularSpeed() + dw_max){
			cout << "return " << 1 << endl;
            return 1;//getWheelTurn(alpha-dw_max);
        }
        else{
			cout << "return " << -1 << endl;
            return -1;//getWheelTurn(-(alpha-dw_min));
        }
    }
}

void MyStrategy::move(const Car& self, const World& world, const Game& game, Move& move) {
    if(way.size() == 0) {
        cout << "Way size is zero" << endl;
        way = bestPath(self, world, game);
    }
    if(curr_index == way.size()) {
        curr_index = 0;
    }

    if(world.getTick() > removeStun) {
        isStunned = false;
    }

    Point current_tile = CurrentTile(self);
    cout << "Current index is ";
    cout << curr_index << endl;
    cout << "Current tile is ";
    current_tile.Print();


    if(curr_index != way.size() && current_tile == way[curr_index]) {
        curr_index++;
        additional_way.clear();
        lostWay = false;
    } else if(lostWay && additional_way[additional_index] == current_tile) {
        additional_index++;
    }

    if((!lostWay && curr_index > 0 && current_tile != way[curr_index - 1]) || (lostWay && additional_index > 0 && current_tile != additional_way[additional_index - 1])) {
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
    }

    int xCoord = lostWay ? additional_way[additional_index].x_ * 800 + 0.5 * game.getTrackTileSize() : way[curr_index].x_ * 800 + 0.5 * game.getTrackTileSize();
    int yCoord = lostWay ? additional_way[additional_index].y_ * 800 + 0.5 * game.getTrackTileSize() : way[curr_index].y_ * 800 + 0.5 * game.getTrackTileSize();

    if(lostWay) {
        SetCoords(xCoord, yCoord, additional_way, additional_index, move);
    } else {
        SetCoords(xCoord, yCoord, way, curr_index, move);
    }

    double angle = self.getAngleTo(xCoord, yCoord);
    double angleCar = self.getAngle();
    double engPower = self.getEnginePower();
    cout << xCoord << " " << yCoord << " " << angle << " " << GetSpeed(self) << " " << engPower << endl;
    move.setEnginePower(1.0);

    for(auto car : world.getCars()) {
        if(car.getId() != self.getId() && abs(self.getAngleTo(car)) < 0.02)
            move.setThrowProjectile(true);
    }

    speedVector.push_back(GetSpeed(self));

    if (world.getTick() > game.getInitialFreezeDurationTicks()) {

        if(lengthOfTheLine(curr_index, way) > 5) {
            //TODO: something about angle (should be close to pi*n/2)
            if(abs(self.getAngleTo(xCoord, yCoord)) < 0.1 && world.getTick() > 100) {
                move.setUseNitro(true);
            }
        }

        if(lengthOfTheLine(curr_index, way) < 3 && GetSpeed(self) > 15) {
            //TODO: something about angle (should be close to pi*n/2)
            //move.setUseNitro(true);
            move.setBrake(true);
        }


        if(!isStunned) {
            if(isgreater(engPower, 0.0)) {
                move.setWheelTurn(getWheelTurnW(angle, self, game));
				cout<<"w==" << self.getAngularSpeed()<<endl;
//                if(needLeft(angle)) {
//                    move.setWheelTurn(-1.0);
//                } else {
//                    move.setWheelTurn(1.0);
//                }

            } else {
                if(needLeft(angle)) {
                    move.setWheelTurn(1.0);
                } else {
                    move.setWheelTurn(-1.0);
                }
            }
            if(std::abs(angle) < 0.1) {
                move.setWheelTurn(0.0);
            }

            if(std::abs(angle) > 1.3 && GetSpeed(self) > 5) {
                move.setBrake(true);
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


    if (world.getTick() > game.getInitialFreezeDurationTicks() + 10) {
        if(LastFiveAroundZero(speedVector)) {
            removeStun = world.getTick() + 25;
            isStunned = true;
        }
    }


}

MyStrategy::MyStrategy() {
    curr_index = 0;
    additional_index = 0;
    lostWay = false;
    isStunned = false;
    removeStun = -1;
}

