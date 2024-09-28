/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME:
 * ANDREW ID:
 * LAST UPDATE:
 *
 * This file keeps track of where the turtle is in the maze
 * and updates the location when the turtle is moved. It shall not
 * contain the maze solving logic/algorithm.
 *
 * This file is used along with student_turtle.cpp. student_turtle.cpp shall
 * contain the maze solving logic/algorithm and shall not make use of the
 * absolute coordinates or orientation of the turtle.
 *
 * This file shall call studentTurtleStep(..) in student_turtle.cpp to determine
 * the next move the turtle will make, and shall use translatePos(..) and
 * translateOrnt(..) to translate this move into absolute coordinates
 * to display the turtle.
 *
 */

#include "student.h"

// Define the maze size
const int32_t MAP_SIZE = 12;

// File-static array to keep track of the number of visits to each cell
static int visitCount[MAP_SIZE][MAP_SIZE] = {0};

/*
 * This procedure takes the current turtle position and orientation and returns true=accept changes, false=do not accept changes
 * Ground rule -- you are only allowed to call the three helper functions defined in student.h, and NO other turtle methods or maze methods (no peeking at the maze!)
 * This file interfaces with functions in student_turtle.cpp
 */
bool moveTurtle(QPointF& pos_, int& new_orientation)
{
  position currentPos;
  currentPos.x = static_cast<int32_t>(pos_.x());
  currentPos.y = static_cast<int32_t>(pos_.y());

  bool bumped = checkBumped(currentPos, new_orientation); // Replace with your own procedure
  turtleMove nextMove = studentTurtleStep(bumped); // define your own turtleMove enum or structure
  pos_ = translatePos(pos_, nextMove);
  new_orientation = translateOrnt(new_orientation, nextMove);

  // REPLACE THE FOLLOWING LINE IN PROJECT 5
  // return studentMoveTurtle(pos_, new_orientation);
  return !atend(currentPos.x, currentPos.y);
}

/*
 * Takes a position and a turtleMove and returns a new position
 * based on the move
 */
QPointF translatePos(QPointF pos_, turtleMove nextMove, int32_t new_orientation) {
  static const int32_t MOVE = 1;
  switch (nextMove) {
  case TURN_LEFT:
    return pos_;
  case TURN_RIGHT:
    updateVisits(pos_);
    return pos_;
  case MOVE:
    if (new_orientation == DOWN) {
      pos_.setY(pos_.y() - MOVE);
    }
    if (new_orientation == RIGHT) {
      pos_.setX(pos_.x() + MOVE);
    }
    if (new_orientation == UP) {
      pos_.setY(pos_.y() + MOVE);
    }
    if (new_orientation == LEFT) {
      pos_.setX(pos_.x() - MOVE);
    }
    return pos_;
  case NO_MOVE:
    return pos_;
  default:
    ROS_ERROR("Invalid move");
    return pos_;
  }
}

/*
 * Takes an orientation and a turtleMove and returns a new orienation
 * based on the move
 */
int translateOrnt(int orientation, turtleMove nextMove) {
  const int32_t NUM_DIRECTIONS = 4;
  const int32_t CLOCKWISE_INCREMENT = 1;
  const int32_t COUNTERCLOCKWISE_INCREMENT = 3;
  switch (nextMove) {
  case TURN_LEFT:
    return (orientation + COUNTERCLOCKWISE_INCREMENT) % NUM_DIRECTIONS;
  case TURN_RIGHT:
    return (orientation + CLOCKWISE_INCREMENT) % NUM_DIRECTIONS;
  case MOVE:
    return orientation;
  case NO_MOVE:
    return orientation;
  default:
    ROS_ERROR("Invalid move");
    return orientation;
  }
}

void checkBumped(position pos_, int32_t &new_orientation) {
  static const int32_t MOVE = 1;

  position currentPos, nextPos;
  currentPos.x = pos_.x;
  currentPos.y = pos_.y;
  nextPos.x = pos_.x;
  nextPos.y = pos_.y;

  switch (new_orientation) {
  case LEFT:
    nextPos.y += MOVE;
    break;
  case DOWN:
    nextPos.x += MOVE;
    break;
  case RIGHT:
    nextPos.x += MOVE;
    nextPos.y += MOVE;
    currentPos.x += MOVE;
    break;
  case UP:
    nextPos.x += MOVE;
    nextPos.y += MOVE;
    currentPos.y += MOVE;
    break;
  default:
    ROS_ERROR("Invalid orientation when checking bump");
    break;
  }

  return bumped(currentPos.x, currentPos.y, nextPos.x, nextPos.y);
}

// Function to update the visit count and call displayVisits
void updateVisits(position pos) {
  if (pos.x >= 0 && pos.x < MAP_SIZE && pos.y >= 0 && pos.y < MAP_SIZE) {
    visitCount[pos.x][pos.y]++;
    displayVisits(visitCount[pos.x][pos.y]);
  }
}

// Getter for visit count
int32_t getVisitCount(position pos) {
    if (pos.x >= 0 && pos.x < MAP_SIZE && pos.y >= 0 && pos.y < MAP_SIZE) {
        return visitCount[pos.x][pos.y];
    }
    return -1; // Return -1 for invalid coordinates
}

// Setter for visit count
void setVisitCount(position pos, int32_t count) {
    if (pos.x >= 0 && pos.x < MAP_SIZE && pos.y >= 0 && pos.y < MAP_SIZE) {
        visitCount[pos.x][pos.y] = count;
    }
}