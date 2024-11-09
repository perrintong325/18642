/*
 * 18-642 Unit Testing Example
 * Example code by Milda Zizyte
 */

#include <iostream>
#include <map> 
#include <cstdint>
#include <limits> 

enum move_state {MOVE_FORWARD, MOVE_BACK};
enum orientation {UP, DOWN};
enum turtleMove { TURN_LEFT, TURN_RIGHT, MOVE, NO_MOVE };
move_state moveTurtle(move_state curr_state, bool atEnd);

// Mock student_maze functions
void setOrientation(orientation);
bool will_bump();

// struct to store the position of the turtle
struct Position {
  int32_t x;
  int32_t y;
};

// typedef for the position struct
typedef Position position;

// enum for the direction the turtle is facing
enum direction { WEST = 0, SOUTH = 1, EAST = 2, NORTH = 3, NA = -1 };

// enum for the state of the turtle
// enum state { GO = 1, CHECKBUMP = 0 };
enum state { FORWARD = 1, RIGHT = 2, LEFT = 3, STOP = 4 };

enum p7_state { SOLVING = 1, LEASTVISIT = 2, RHR = 3, SOLVED = 4 };

void ROS_ERROR(std::string e);

void ROS_INFO(const char* format, ...);

void nextState(int32_t &moving_state, bool bump, int32_t orientation,
               bool stopMove,
               std::map<int32_t, int32_t> &surroundingPos,
               int32_t &minVisitDirection, int32_t &currentState);

void leastVisitNextState(int32_t &moving_state, bool bump, int32_t orientation,
                         int32_t &minVisitDirection,
                         std::map<int32_t, int32_t> &surroundingPos);

void RHRnextState(int32_t &moving_state, bool bump);

void displayVisits(int visits);

bool checkError();

void resetError();