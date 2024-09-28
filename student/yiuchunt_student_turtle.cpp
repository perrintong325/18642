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
// turtleMove studentTurtleStep(bool bumped) { return MOVE; }

// OK TO MODIFY BELOW THIS LINE

// Define the maze size
const int32_t MAZE_SIZE = 23;
const int32_t MAZE_CENTER = MAZE_SIZE / 2;

// File-static array to keep track of the number of visits to each cell
static int visitCount[MAZE_SIZE][MAZE_SIZE] = {0};

// this procedure takes the current turtle position and orientation and returns
// true=the new orientation will hit a wall, false=the new orientation is safe
// and updates bump to true if the turtle bumped into a wall
// void checkBumped(position pos_, int32_t &new_orientation, bool &bump) {
//   static const int32_t MOVE = 1;

//   position currentPos, nextPos;
//   currentPos.x = pos_.x;
//   currentPos.y = pos_.y;
//   nextPos.x = pos_.x;
//   nextPos.y = pos_.y;

//   switch (new_orientation) {
//   case LEFT:
//     nextPos.y += MOVE;
//     break;
//   case DOWN:
//     nextPos.x += MOVE;
//     break;
//   case RIGHT:
//     nextPos.x += MOVE;
//     nextPos.y += MOVE;
//     currentPos.x += MOVE;
//     break;
//   case UP:
//     nextPos.x += MOVE;
//     nextPos.y += MOVE;
//     currentPos.y += MOVE;
//     break;
//   default:
//     ROS_ERROR("Invalid orientation when checking bump");
//     break;
//   }

//   bump = bumped(currentPos.x, currentPos.y, nextPos.x, nextPos.y);
//   return;
// }

// this procedure takes the current orientation and a boolean indicating
// whether to rotate clockwise or counterclockwise, and returns the new
// orientation
void rotateDirection(int32_t &new_orientation, bool clockwise) {
  const int32_t NUM_DIRECTIONS = 4;
  const int32_t CLOCKWISE_INCREMENT = 1;
  const int32_t COUNTERCLOCKWISE_INCREMENT = 3;
  if (clockwise) {
    new_orientation = (new_orientation + CLOCKWISE_INCREMENT) % NUM_DIRECTIONS;
  } else {
    new_orientation =
        (new_orientation + COUNTERCLOCKWISE_INCREMENT) % NUM_DIRECTIONS;
  }
  return;
}
// Function to update the visit count and call displayVisits
void updateVisits(position pos) {
  if (pos.x >= 0 && pos.x < MAZE_SIZE && pos.y >= 0 && pos.y < MAZE_SIZE) {
    visitCount[pos.x][pos.y]++;
    // displayVisits(visitCount[pos.x][pos.y]);
  }
}

// Getter for visit count
int32_t getVisitCount(position pos) {
    if (pos.x >= 0 && pos.x < MAZE_SIZE && pos.y >= 0 && pos.y < MAZE_SIZE) {
        return visitCount[pos.x][pos.y];
    }
    return -1; // Return -1 for invalid coordinates
}

// Setter for visit count
void setVisitCount(position pos, int32_t count) {
    if (pos.x >= 0 && pos.x < MAZE_SIZE && pos.y >= 0 && pos.y < MAZE_SIZE) {
        visitCount[pos.x][pos.y] = count;
    }
}

// this procedure takes the current state, orientation, and a boolean indicating
// whether the turtle bumped into a wall, and returns the next state
// void nextState(int32_t &current_state, int32_t &new_orientation, bool bump,
//                position pos) {
//   // if went straight last cycle turn right to find new path
//   switch (current_state) {
//   case GO: // if in GO state, turn right to find new path
//     rotateDirection(new_orientation, true);
//     current_state = CHECKBUMP;
//     updateVisits(pos);
//     break;
//   case CHECKBUMP:
//     if (bump) { // if bumped after turn undo turn/turn left
//       rotateDirection(new_orientation, false);
//     } else { // if there's no bump, go straight
//       current_state = GO;
//     }
//     break;
//   default:
//     ROS_ERROR("Invalid state");
//     break;
//   }
//   return;
// }

void nextState(int32_t &current_state, bool bump) {
  switch (current_state) {
  case RIGHT:
    if (bump) {
      current_state = LEFT;
    } else {
      current_state = FORWARD;
    }
    break;
  case LEFT:
    if (bump) {
      current_state = LEFT;
    } else {
      current_state = FORWARD;
    }
    break;
  case FORWARD:
    current_state = RIGHT;
    break;
  default:
    ROS_ERROR("Invalid state");
    break;
  }
}

// this procedure takes the current turtle position, orientation, state, and
// solved status, and updates the turtle position
void moveTurtle(QPointF &pos_, int32_t &new_orientation, int32_t &current_state,
                bool &solved) {
  static const int32_t MOVE = 1;
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
  case CHECKBUMP:
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
turtleMove studentTurtleStep(bool bumped) {
// bool studentMoveTurtle(QPointF &pos_, int32_t &new_orientation) {
  static const int32_t TIMEOUT =
      4; // bigger number slows down simulation so you can see what's happening
  static const int32_t CYCLE_DECREASE = 1;
  static int32_t cycle = 0;
  static int32_t current_state = RIGHT;
  // static bool solved = false;
  // static bool bump = true;
  // static bool initVisit = true;
  static position currentPos = {MAZE_CENTER, MAZE_CENTER};
  static new_orientation = UP;

  ROS_INFO("Turtle update Called  cycle=%d", cycle);
  if (cycle == 0) {
    // position currentPos, mapPos;
    // currentPos.x = static_cast<int32_t>(pos_.x());
    // currentPos.y = static_cast<int32_t>(pos_.y());

    // Update the visit count and call displayVisits
    // mapPos.x = MAZE_CENTER + currentPos.x;
    // mapPos.y = MAZE_CENTER + currentPos.y;

    // checkBumped(currentPos, new_orientation, bump);
    // solved = atend(currentPos.x, currentPos.y);

    nextState(current_state, bump);

    ROS_INFO("Orientation=%d  STATE=%d", new_orientation, current_state);

    // moveTurtle(pos_, new_orientation, current_state, solved);
    cycle = TIMEOUT;
    switch(current_state) {
      case FORWARD:
        if (new_orientation == DOWN) {
          currentPos.y -= 1;
        }
        if (new_orientation == RIGHT) {
          currentPos.x += 1;
        }
        if (new_orientation == UP) {
          currentPos.y += 1;
        }
        if (new_orientation == LEFT) {
          currentPos.x -= 1;
        }
        return MOVE;
      case RIGHT:
        updateVisits(currentPos);
        rotateDirection(new_orientation, true);
        return TURN_RIGHT;
      case LEFT:
        rotateDirection(new_orientation, false);
        return TURN_LEFT;
      default:
        ROS_ERROR("Invalid state");
        break;
    }
    // cycle = TIMEOUT;
    // return true; // submit changes
  }
  cycle -= CYCLE_DECREASE; // decrease cycle
  return NO_MOVE;          // don't submit changes
  // if (solved) {
  //   return false; // don't submit changes
  // } else {
  //   cycle -= CYCLE_DECREASE; // decrease cycle
  //   return false;            // don't submit changes
  // }
}
