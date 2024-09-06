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

#define TIMEOUT                                                                \
  40 // bigger number slows down simulation so you can see what's happening
float w, cs;
float fx1, fy1, fx2, fy2;
float z, aend, mod, bp, q;

enum direction { LEFT = 0, DOWN = 1, RIGHT = 2, UP = 3 };
enum state {
  GO = 2,
  CHECKBP = 1
};

// this procedure takes the current turtle position and orientation and returns
// true=submit changes, false=do not submit changes
// Ground rule -- you are only allowed to call the helper functions "bumped(..)"
// and "atend(..)", and NO other turtle methods or maze methods (no peeking at
// the maze!)
bool studentMoveTurtle(QPointF &pos_, int &nw_or) {
  ROS_INFO("Turtle update Called  w=%f", w);
  mod = true;
  if (w == 0) {
    fx1 = pos_.x();
    fy1 = pos_.y();
    fx2 = pos_.x();
    fy2 = pos_.y();
    switch (nw_or) {
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
    }
    bp = bumped(fx1, fy1, fx2, fy2);
    aend = atend(pos_.x(), pos_.y());
    // if went straight last cycle turn right to find new path
    if (cs == GO) {        
      switch (nw_or) {
      case LEFT:
        nw_or = DOWN;
        break;
      case DOWN:
        nw_or = RIGHT;
        break;
      case RIGHT:
        nw_or = UP;
        break;
      case UP:
        nw_or = LEFT;
        break;
      }
      cs = CHECKBP;
    } else if (bp) {      // if bumped in after turn undo turn 
      switch (nw_or) {
      case LEFT:
        nw_or = UP;
        break;
      case DOWN:
        nw_or = LEFT;
        break;
      case RIGHT:
        nw_or = DOWN;
        break;
      case UP:
        nw_or = RIGHT;
        break;
      }
    } else {              // if no bump in cs 1 or 0 go straight
      cs = GO;
    }
    ROS_INFO("Orientation=%f  STATE=%f", nw_or, cs);
    z = (cs == GO);
    mod = true;

    if (z == true && aend == false) {
      if (nw_or == DOWN)
        pos_.setY(pos_.y() - 1);
      if (nw_or == RIGHT)
        pos_.setX(pos_.x() + 1);
      if (nw_or == UP)
        pos_.setY(pos_.y() + 1);
      if (nw_or == LEFT)
        pos_.setX(pos_.x() - 1);
      z = false;
      mod = true;
    }
  }
  if (aend)
    return false;
  if (w == 0)
    w = TIMEOUT;
  else
    w -= 1;
  if (w == TIMEOUT)
    return true;
  return false;
}
