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

struct Position {
  float x;
  float y;
};

typedef Position position;

enum direction { LEFT = 0, DOWN = 1, RIGHT = 2, UP = 3 };
enum state { GO = 1, CHECKBP = 0 };

// this procedure takes the current turtle position and orientation and returns
// true=the new orientation will hit a wall, false=the new orientation is safe
void checkBumped(QPointF &pos_, int &new_orientation, bool &bump) {
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
    break;
  }

  bump = bumped(pos1.x, pos1.y, pos2.x, pos2.y);
}

void rotateLeft(int &new_orientation) {
  switch (new_orientation) {
  case LEFT:
    new_orientation = DOWN;
    break;
  case DOWN:
    new_orientation = RIGHT;
    break;
  case RIGHT:
    new_orientation = UP;
    break;
  case UP:
    new_orientation = LEFT;
    break;
  default:
    ROS_ERROR("Invalid orientation when rotating left");
    break;
  }
  return;
}

void rotateRight(int &new_orientation) {
  switch (new_orientation) {
  case LEFT:
    new_orientation = UP;
    break;
  case DOWN:
    new_orientation = LEFT;
    break;
  case RIGHT:
    new_orientation = DOWN;
    break;
  case UP:
    new_orientation = RIGHT;
    break;
  default:
    ROS_ERROR("Invalid orientation when rotating right");
    break;
  }
  return;
}

void nextState(int &current_state, int &new_orientation, bool bump) {
  // if went straight last cycle turn right to find new path
  switch (current_state) {
  case GO:
    rotateRight(new_orientation);
    current_state = CHECKBP;
    break;
  case CHECKBP:
    if (bump) { // if bumped after turn undo turn/turn left
      rotateLeft(new_orientation);
    } else { // if there's no bump, go straight
      current_state = GO;
    }
    break;
  default:
    break;
  }
  return;
}

void moveTurtle(QPointF &pos_, int &new_orientation, int &current_state,
                bool &solved) {
  switch (current_state) {
    case GO: // move in the new orientation if in GO state
      if (solved == false) {
        if (new_orientation == DOWN) {
          pos_.setY(pos_.y() - 1);
        }
        if (new_orientation == RIGHT) {
          pos_.setX(pos_.x() + 1);
        }
        if (new_orientation == UP) {
          pos_.setY(pos_.y() + 1);
        }
        if (new_orientation == LEFT) {
          pos_.setX(pos_.x() - 1);
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
bool studentMoveTurtle(QPointF &pos_, int &new_orientation) {
  static const int TIMEOUT =
      4; // bigger number slows down simulation so you can see what's happening
  static int cycle = 0;
  static int current_state = CHECKBP;
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
    cycle -= 1;
    return false;
  }
}
