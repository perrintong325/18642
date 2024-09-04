/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Perrin Tong  
 * ANDREW ID: yiuchunt
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

enum direction {
  UP = 0,
  RIGHT = 1,
  DOWN = 2,
  LEFT = 3
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

    switch(nw_or) {
      case UP:
        fy2 += 1;
        break;
      case RIGHT:
        fx2 += 1;
        break;
      case DOWN:
        // fx2 += 1;
        // fy2 += 1;
        // fx1 += 1;
        fy1 -= 1;
        break;
      case LEFT:
        // fx2 += 1;
        // fy2 += 1;
        // fy1 += 1;
        fx1 -= 1;
        break;
    }

    bp = bumped(fx1, fy1, fx2, fy2);
    aend = atend(pos_.x(), pos_.y());

    if (cs == 2) {
      switch(nw_or) {
        case UP:
          nw_or = RIGHT;
          break;
        case RIGHT:
          nw_or = DOWN;
          break;
        case DOWN:
          nw_or = LEFT;
          break;
        case LEFT:
          nw_or = UP;
          break;
      }
      cs = 1;
    } else if (bp) {
      switch(nw_or) {
        case UP:
          nw_or = LEFT;
          break;
        case RIGHT:
          nw_or = UP;
          break;
        case DOWN:
          nw_or = RIGHT;
          break;
        case LEFT:
          nw_or = DOWN;
          break;
      }
      cs = 0;
    } else {
      cs = 2;
    }
    ROS_INFO("Orientation=%f  STATE=%f", nw_or, cs);
    z = (cs == 2);
    mod = true;
    if (z == true && aend == false) {
      if (nw_or == RIGHT)
        pos_.setY(pos_.y() - 1);
      if (nw_or == DOWN)
        pos_.setX(pos_.x() + 1);
      if (nw_or == LEFT)
        pos_.setY(pos_.y() + 1);
      if (nw_or == UP)
        pos_.setX(pos_.x() - 1);
      z = false;
      mod = true;
    }
  }
  if (aend)
    return false;
  if (w == 0) {
    w = TIMEOUT;
    return true;
  } else {
    w -= 1;
    return false;
  }
}
