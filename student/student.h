#include <ros/ros.h>
#include <boost/bind.hpp>
#include <ece642rtle/timeInt8.h>
#include <std_msgs/Empty.h>
#include <ece642rtle/RTIbump.h>
#include <ece642rtle/RTIatend.h>
#include <ece642rtle/PoseOrntBundle.h>
#include <ece642rtle/bumpEcho.h>
#include <ece642rtle/aendEcho.h>
#include <QPointF>

// Functions to interface with ROS. Don't change these lines!
bool bumped(int x1,int y1,int x2,int y2);
bool atend(int x, int y);
void displayVisits(int visits);
bool moveTurtle(QPointF& pos_, int& nw_or);

// Scope-preserving changes to these lines permitted (see p5 writeup)
enum turtleMove { TURN_LEFT, TURN_RIGHT, MOVE, NO_MOVE };
QPointF translatePos(QPointF pos_, turtleMove nextMove);
int translateOrnt(int orientation, turtleMove nextMove);
turtleMove studentTurtleStep(bool bumped);

// OK to change below this line
bool studentMoveTurtle(QPointF& pos_, int& nw_or);

// struct to store the position of the turtle
struct Position {
  int32_t x;
  int32_t y;
};

// typedef for the position struct
typedef Position position;

// enum for the direction the turtle is facing
enum direction { LEFT = 0, DOWN = 1, RIGHT = 2, UP = 3 };

// enum for the state of the turtle
// enum state { GO = 1, CHECKBUMP = 0 };
enum state { FORWARD = 1, RIGHT = 2, LEFT = 3}


