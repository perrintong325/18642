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

#ifdef testing
#include "student_mock.h"
#endif
#ifndef testing
#include "student.h"
#include "ros/ros.h"
#endif

// Ignore this line until project 5
// turtleMove studentTurtleStep(bool bumped) { return MOVE; }

// OK TO MODIFY BELOW THIS LINE

// Define the maze size
const int32_t MAZE_SIZE = 23;
const int32_t MAZE_CENTER = MAZE_SIZE / 2;

// File-static array to keep track of the number of visits to each cell
static int visitCount[MAZE_SIZE][MAZE_SIZE] = {0};

// this procedure takes the current orientation and a boolean indicating
// whether to rotate clockwise or counterclockwise, and returns the new
// orientation
void rotateDirection(int32_t &orientation, bool clockwise) {
  const int32_t NUM_DIRECTIONS = 4;
  const int32_t CLOCKWISE_INCREMENT = 1;
  const int32_t COUNTERCLOCKWISE_INCREMENT = 3;
  if (clockwise) {
    orientation = (orientation + CLOCKWISE_INCREMENT) % NUM_DIRECTIONS;
  } else {
    orientation = (orientation + COUNTERCLOCKWISE_INCREMENT) % NUM_DIRECTIONS;
  }
  return;
}
// Function to update the visit count and call displayVisits
void updateVisitsLocal(position pos) {
  if (pos.x >= 0 && pos.x < MAZE_SIZE && pos.y >= 0 && pos.y < MAZE_SIZE) {
    visitCount[pos.x][pos.y]++;
  }
  displayVisits(visitCount[pos.x][pos.y]);
}

// Getter for visit count
int32_t getVisitCountLocal(position pos) {
  if (pos.x >= 0 && pos.x < MAZE_SIZE && pos.y >= 0 && pos.y < MAZE_SIZE) {
    return visitCount[pos.x][pos.y];
  }
  return -1; // Return -1 for invalid coordinates
}

// Function to get the visit count of surrounding cells
std::map<int32_t, int32_t> getSurroundingPos(position pos) {
  std::map<int32_t, int32_t> surroundingPos;
  surroundingPos[NORTH] = getVisitCountLocal({pos.x, pos.y + 1});
  surroundingPos[EAST] = getVisitCountLocal({pos.x + 1, pos.y});
  surroundingPos[SOUTH] = getVisitCountLocal({pos.x, pos.y - 1});
  surroundingPos[WEST] = getVisitCountLocal({pos.x - 1, pos.y});
  return surroundingPos;
}

// Setter for visit count
void setVisitCountLocal(position pos, int32_t count) {
  if (pos.x >= 0 && pos.x < MAZE_SIZE && pos.y >= 0 && pos.y < MAZE_SIZE) {
    visitCount[pos.x][pos.y] = count;
  }
}

// Function to calculate the minimum turns to reach the new orientation
int calculateTurns(int current_orientation, int orientation) {
  int turns = std::abs(current_orientation - orientation);
  return std::min(turns, 4 - turns); // Minimum turns considering wrap-around
}

// Function to get the find the direction with the least visit count, return -1
// if all surrounding cells have the same visit count
int32_t getMinVisitDirection(std::map<int32_t, int32_t> surroundingVisitCounts,
                             int32_t orientation, int32_t state) {
  int32_t minVisit = std::numeric_limits<int32_t>::max();
  int32_t minOrientation = NA;

  bool allSame = true;
  int32_t firstVisitCount = -1;
  for (const auto &entry : surroundingVisitCounts) {
    int32_t visitCount = entry.second;
    if (firstVisitCount == -1) {
      firstVisitCount = visitCount;
    } else if (visitCount != firstVisitCount) {
      allSame = false;
      break;
    }
  }

  if (allSame) {
    return NA;
  }

  for (const auto &entry : surroundingVisitCounts) {
    if (entry.second < minVisit) {
      minVisit = entry.second;
      minOrientation = entry.first;
    } else if (entry.second == minVisit) {
      // If visit count is the same, choose the orientation with the least turns
      int currentTurns = calculateTurns(orientation, minOrientation);
      int newTurns = calculateTurns(orientation, entry.first);
      if (newTurns < currentTurns) {
        minOrientation = entry.first;
      } else if (newTurns == currentTurns) {
        int offset = 0;
        switch (state) {
        case RIGHT:
          offset = -1;
          break;
        case LEFT:
          offset = 1;
          break;
        case FORWARD:
        case STOP:
        default:
          ROS_ERROR("Invalid state");
          break;
        }
        minOrientation = (minOrientation + offset) % 4;
      }
    }
  }

  return minOrientation;
}

// When not all surrounding cells have same visit count, determine next state
// based on least visit count, if bump, try to visit the next least visited cell
void leastVisitNextState(int32_t &moving_state, bool bump, int32_t orientation,
                         int32_t &minVisitDirection,
                         std::map<int32_t, int32_t> &surroundingPos) {
  switch (moving_state) {
  case RIGHT:
    if (orientation == minVisitDirection) {
      if (bump) { // T2.1
        surroundingPos[minVisitDirection] = std::numeric_limits<int32_t>::max();
        minVisitDirection = getMinVisitDirection(surroundingPos, orientation, LEFT);
        moving_state = LEFT;
      } else { // T2.7
        moving_state = FORWARD;
      }
    } else { // T2.4
      moving_state = LEFT;
    }
    break;
  case LEFT:
    if (orientation == minVisitDirection) {
      if (bump) { // T2.3
        surroundingPos[minVisitDirection] = std::numeric_limits<int32_t>::max();
        minVisitDirection = getMinVisitDirection(surroundingPos, orientation, LEFT);
        moving_state = LEFT;
      } else { // T2.6
        moving_state = FORWARD;
      }
    } else { // T2.5
      moving_state = LEFT;
    }
    break;
  case FORWARD:
    break; //T4
  case STOP:
    moving_state = RIGHT;
    break; //T2
  default:
    ROS_ERROR("Invalid state");
    break;
  }
}

// Right hand rule, for when all surrounding cells have same visit count
void RHRnextState(int32_t &moving_state, bool bump) {
  switch (moving_state) {
  case RIGHT:
    if (bump) { // T1.1
      moving_state = LEFT;
    } else { // T1.4
      moving_state = FORWARD;
    }
    break;
  case LEFT:
    if (bump) { // T1.2
      moving_state = LEFT;
    } else { // T1.3
      moving_state = FORWARD;
    }
    break;
  case FORWARD:
    break; //T5
  case STOP:
    moving_state = RIGHT; //T1
    break;
  default:
    ROS_ERROR("Invalid state");
    break;
  }
}

// State Chart Top Level Function (state block)
void nextState(int32_t &moving_state, bool bump, int32_t orientation,
               bool stopMove,
               std::map<int32_t, int32_t> &surroundingPos,
               int32_t &minVisitDirection, int32_t &currentState) {

  switch (currentState) {
  case SOLVING: // S1
    moving_state = STOP;
    if (!stopMove) {
      if (minVisitDirection == NA) { // T1
        currentState = RHR;
      } else { // T2
        currentState = LEASTVISIT;
      }
    } else { // T3
      currentState = SOLVED;
    }
    break;
  case RHR: // S2
    RHRnextState(moving_state, bump);
    if (moving_state == FORWARD) { // T5
      currentState = SOLVING;
    }
    break;
  case LEASTVISIT: // S3
    leastVisitNextState(moving_state, bump, orientation, minVisitDirection,
                        surroundingPos);
    if (moving_state == FORWARD) { // T4
      currentState = SOLVING;
    }
    break;
  case SOLVED: // S4
    moving_state = STOP;
    currentState = SOLVED; // T6
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
turtleMove studentTurtleStep(bool bumped, bool stopMove) {
  static const int32_t TIMEOUT =
      10; // bigger number slows down simulation so you can see what's happening
  static const int32_t CYCLE_DECREASE = 1;
  static int32_t cycle = 0;
  static int32_t moving_state = RIGHT;
  static position currentPos = {MAZE_CENTER, MAZE_CENTER};
  static int32_t orientation = NORTH;
  static std::map<int32_t, int32_t> surroundingPos;
  static int32_t minVisitDirection = NA;
  static int32_t currentState = SOLVING;

  ROS_INFO("Turtle update Called  cycle=%d", cycle);
  turtleMove move = NO_MOVE;
  if (cycle == 0) {
    if (currentState == SOLVING) {
      surroundingPos = getSurroundingPos(currentPos);
      minVisitDirection = getMinVisitDirection(surroundingPos, orientation, RIGHT);
    }
    nextState(moving_state, bumped, orientation, stopMove,
              surroundingPos, minVisitDirection, currentState);

    ROS_INFO("Orientation=%d  STATE=%d", orientation, moving_state);

    cycle = TIMEOUT;
    switch (moving_state) {
    case FORWARD:
      if (orientation == SOUTH) {
        currentPos.y -= 1;
      }
      if (orientation == EAST) {
        currentPos.x += 1;
      }
      if (orientation == NORTH) {
        currentPos.y += 1;
      }
      if (orientation == WEST) {
        currentPos.x -= 1;
      }
      updateVisitsLocal(currentPos);
      move = MOVE;
      break;
    case RIGHT:
      rotateDirection(orientation, true);
      move = TURN_RIGHT;
      break;
    case LEFT:
      rotateDirection(orientation, false);
      move = TURN_LEFT;
      break;
    case STOP:
      move = NO_MOVE;
      break;
    default:
      ROS_ERROR("Invalid state");
      break;
    }
  }
  if (!stopMove) {
    cycle -= CYCLE_DECREASE; // decrease cycle
  }
  return move;
}
