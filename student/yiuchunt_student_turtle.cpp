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

const int TIMEOUT =
    4; // bigger number slows down simulation so you can see what's happening
int cycle, current_state, prev_state;
int fx1, fy1, fx2, fy2;
int solved, bump;

enum direction { LEFT = 0, DOWN = 1, RIGHT = 2, UP = 3 };
enum state { GO = 1, CHECKBP = 0 };

// this procedure takes the current turtle position and orientation and returns
// true=submit changes, false=do not submit changes
// Ground rule -- you are only allowed to call the helper functions "bumped(..)"
// and "atend(..)", and NO other turtle methods or maze methods (no peeking at
// the maze!)
bool studentMoveTurtle(QPointF &pos_, int &new_or) {
  ROS_INFO("Turtle update Called  cycle=%f", cycle);
  if (cycle == 0) {
    fx1 = pos_.x();
    fy1 = pos_.y();
    fx2 = pos_.x();
    fy2 = pos_.y();

    switch (new_or) {
    case LEFT:
      fy2 += 1;
      break;
    case DOWN:
      fx2 += 1;
      break;
    case RIGHT:
      fx2 += 1;
      fy2 += 1;
      fx1 += 1;
      break;
    case UP:
      fx2 += 1;
      fy2 += 1;
      fy1 += 1;
      break;
    default:
      break;
    }

    bump = bumped(fx1, fy1, fx2, fy2);
    solved = atend(pos_.x(), pos_.y());

    // if went straight last cycle turn right to find new path
    if (prev_state == GO) {
      switch (new_or) {
      case LEFT:
        new_or = DOWN;
        break;
      case DOWN:
        new_or = RIGHT;
        break;
      case RIGHT:
        new_or = UP;
        break;
      case UP:
        new_or = LEFT;
        break;
      default:
        break;
      }
      current_state = CHECKBP;
    } else {      // previous state here is checkbp
      if (bump) { // if bumped after turn undo turn/turn left
        switch (new_or) {
        case LEFT:
          new_or = UP;
          break;
        case DOWN:
          new_or = LEFT;
          break;
        case RIGHT:
          new_or = DOWN;
          break;
        case UP:
          new_or = RIGHT;
          break;
        default:
          break;
        }
      } else { // if there's no bump, go straight
        current_state = GO;
      }
    }

    ROS_INFO("Orientation=%f  STATE=%f", new_or, current_state);

    if (current_state == GO && solved == false) {
      if (new_or == DOWN)
        pos_.setY(pos_.y() - 1);
      if (new_or == RIGHT)
        pos_.setX(pos_.x() + 1);
      if (new_or == UP)
        pos_.setY(pos_.y() + 1);
      if (new_or == LEFT)
        pos_.setX(pos_.x() - 1);
    }

    prev_state = current_state;
  }

  if (solved)
    return false;

  if (cycle == 0) {
    cycle = TIMEOUT;
    return true;
  } else {
    cycle -= 1;
    return false;
  }
}
