/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME:
 * ANDREW ID:
 * LAST UPDATE:
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the left-hand rule. The code is intentionaly left obfuscated.
 *
 */

#include "student.h"

// Ignore this line until project 5
turtleMove studentTurtleStep(bool bumped) { return MOVE; }

// OK TO MODIFY BELOW THIS LINE

// struct to store the position of the turtle
struct Position {
  float x;
  float y;
};

// typedef for the position struct
typedef Position position;

// enum for the direction the turtle is facing
enum direction { LEFT = 0, DOWN = 1, RIGHT = 2, UP = 3 };

// enum for the state of the turtle
enum state { GO = 1, CHECKBP = 0 };

// this procedure takes the current turtle position and orientation and returns
// true=the new orientation will hit a wall, false=the new orientation is safe
void checkBumped(QPointF &pos_, int8_t &new_orientation, bool &bump) {
  position pos1, pos2;
  pos1.x = pos_.x();
  pos1.y = pos_.y();
  pos2.x = pos_.x();
  pos2.y = pos_.y();

  switch (new_orientation) {
  case LEFT:
    pos2.y += 1;
    break;
  case DOWN:
    pos2.x += 1;
    break;
  case RIGHT:
    pos2.x += 1;
    pos2.y += 1;
    pos1.x += 1;
    break;
  case UP:
    pos2.x += 1;
    pos2.y += 1;
    pos1.y += 1;
    break;
  default:
    ROS_ERROR("Invalid orientation when checking bump");
    break;
  }

  bump = bumped(pos1.x, pos1.y, pos2.x, pos2.y);
  return;
}

// this procedure takes the current orientation and a boolean indicating
// whether to rotate clockwise or counterclockwise, and returns the new
// orientation
void rotateDirection(int8_t &new_orientation, bool clockwise) {
  const int8_t NUM_DIRECTIONS = 4;
  const int8_t CLOCKWISE_INCREMENT = 1;
  const int8_t COUNTERCLOCKWISE_INCREMENT = 3;
  if (clockwise) {
    new_orientation = (new_orientation + CLOCKWISE_INCREMENT) % NUM_DIRECTIONS;
  } else {
    new_orientation =
        (new_orientation + COUNTERCLOCKWISE_INCREMENT) % NUM_DIRECTIONS;
  }
  return;
}

// this procedure takes the current state, orientation, and a boolean indicating
// whether the turtle bumped into a wall, and returns the next state
void nextState(int8_t &current_state, int8_t &new_orientation, bool bump) {
  // if went straight last cycle turn right to find new path
  switch (current_state) {
  case GO: // if in GO state, turn right to find new path
    rotateDirection(new_orientation, true);
    current_state = CHECKBP;
    break;
  case CHECKBP:
    if (bump) { // if bumped after turn undo turn/turn left
      rotateDirection(new_orientation, false);
    } else { // if there's no bump, go straight
      current_state = GO;
    }
    break;
  default:
    ROS_ERROR("Invalid state");
    break;
  }
  return;
}

// this procedure takes the current turtle position, orientation, state, and
// solved status, and updates the turtle position
void moveTurtle(QPointF &pos_, int8_t &new_orientation, int8_t &current_state,
                bool &solved) {
  static const int8_t MOVE = 1;
  switch (current_state) {
  case GO: // move in the new orientation if in GO state
    if (solved == false) {
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
    }
    break;
  case CHECKBP:
    break;
  default:
    ROS_ERROR("Invalid state");
    break;
  }
}

// this procedure takes the current turtle position and orientation and returns
// true=submit changes, false=do not submit changes
// Ground rule -- you are only allowed to call the helper functions "bumped(..)"
// and "atend(..)", and NO other turtle methods or maze methods (no peeking at
// the maze!)
bool studentMoveTurtle(QPointF &pos_, int8_t &new_orientation) {
  static const int8_t TIMEOUT =
      4; // bigger number slows down simulation so you can see what's happening
  static const int8_t CYCLE_DECREASE = 1;
  static int8_t cycle = 0;
  static int8_t current_state = CHECKBP;
  static bool solved = false;
  static bool bump = true;

  ROS_INFO("Turtle update Called  cycle=%f", cycle);
  if (cycle == 0) {
    checkBumped(pos_, new_orientation, bump);
    solved = atend(pos_.x(), pos_.y());

    nextState(current_state, new_orientation, bump);

    ROS_INFO("Orientation=%f  STATE=%f", new_orientation, current_state);

    moveTurtle(pos_, new_orientation, current_state, solved);
    cycle = TIMEOUT;
    return true;
  }

  if (solved) {
    return false;
  } else {
    cycle -= CYCLE_DECREASE;
    return false;
  }
}
