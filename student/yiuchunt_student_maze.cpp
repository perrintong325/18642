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

/*
 * This procedure takes the current turtle position and orientation and returns
 * true=accept changes, false=do not accept changes Ground rule -- you are only
 * allowed to call the three helper functions defined in student.h, and NO other
 * turtle methods or maze methods (no peeking at the maze!) This file interfaces
 * with functions in student_turtle.cpp
 */
bool moveTurtle(QPointF &pos_, int &new_orientation) {
  static bool stopMove = false;
  position currentPos;
  currentPos.x = static_cast<int32_t>(pos_.x());
  currentPos.y = static_cast<int32_t>(pos_.y());

  bool bumped = checkBumped(currentPos, new_orientation);
  turtleMove nextMove = studentTurtleStep(bumped, stopMove);
  pos_ = translatePos(pos_, nextMove, new_orientation);
  new_orientation = translateOrnt(new_orientation, nextMove);

  if (atend(currentPos.x, currentPos.y)) {
    stopMove = true;
    return false;
  }
  return true;
}

/*
 * Takes a position and a turtleMove and returns a new position
 * based on the move
 */
QPointF translatePos(QPointF pos_, turtleMove nextMove,
                     int32_t new_orientation) {
  static const int32_t movement = 1;
  switch (nextMove) {
  case TURN_LEFT:
    return pos_;
  case TURN_RIGHT:
    return pos_;
  case MOVE:
    if (new_orientation == SOUTH) {
      pos_.setY(pos_.y() - movement);
    }
    if (new_orientation == EAST) {
      pos_.setX(pos_.x() + movement);
    }
    if (new_orientation == NORTH) {
      pos_.setY(pos_.y() + movement);
    }
    if (new_orientation == WEST) {
      pos_.setX(pos_.x() - movement);
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

bool checkBumped(position pos_, int32_t &new_orientation) {
  static const int32_t movement = 1;

  position currentPos, nextPos;
  currentPos.x = pos_.x;
  currentPos.y = pos_.y;
  nextPos.x = pos_.x;
  nextPos.y = pos_.y;

  switch (new_orientation) {
  case WEST:
    nextPos.y += movement;
    break;
  case SOUTH:
    nextPos.x += movement;
    break;
  case EAST:
    nextPos.x += movement;
    nextPos.y += movement;
    currentPos.x += movement;
    break;
  case NORTH:
    nextPos.x += movement;
    nextPos.y += movement;
    currentPos.y += movement;
    break;
  default:
    ROS_ERROR("Invalid orientation when checking bump");
    break;
  }

  return bumped(currentPos.x, currentPos.y, nextPos.x, nextPos.y);
}