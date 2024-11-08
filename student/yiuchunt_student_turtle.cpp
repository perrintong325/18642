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
void updateVisitsLocal(position pos) {
  if (pos.x >= 0 && pos.x < MAZE_SIZE && pos.y >= 0 && pos.y < MAZE_SIZE) {
    visitCount[pos.x][pos.y]++;
  }
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
int calculateTurns(int current_orientation, int new_orientation) {
  int turns = std::abs(current_orientation - new_orientation);
  return std::min(turns, 4 - turns); // Minimum turns considering wrap-around
}

// Function to get the find the direction with the least visit count, return -1
// if all surrounding cells have the same visit count
int32_t getMinVisitDirection(std::map<int32_t, int32_t> surroundingVisitCounts,
                             int32_t new_orientation) {
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
      int currentTurns = calculateTurns(new_orientation, minOrientation);
      int newTurns = calculateTurns(new_orientation, entry.first);
      if (newTurns < currentTurns) {
        minOrientation = entry.first;
      }
    }
  }

  return minOrientation;
}

// When not all surrounding cells have same visit count, determine next state
// based on least visit count, if bump, try to visit the next least visited cell
void leastVisitNextState(int32_t &move_state, bool bump,
                         int32_t new_orientation, int32_t &minVisitDirection,
                         std::map<int32_t, int32_t> &surroundingPos) {
  switch (move_state) {
  case RIGHT:
    if (new_orientation == minVisitDirection) {
      if (bump) { // T2.1
        surroundingPos[minVisitDirection] = std::numeric_limits<int32_t>::max();
        minVisitDirection =
            getMinVisitDirection(surroundingPos, new_orientation);
        move_state = LEFT;
      } else { // T2.7
        move_state = FORWARD;
      }
    } else { // T2.4
      move_state = LEFT;
    }
    break;
  case LEFT:
    if (new_orientation == minVisitDirection) {
      if (bump) { // T2.3
        surroundingPos[minVisitDirection] = std::numeric_limits<int32_t>::max();
        minVisitDirection =
            getMinVisitDirection(surroundingPos, new_orientation);
        move_state = LEFT;
      } else { // T2.6
        move_state = FORWARD;
      }
    } else { // T2.5
      move_state = LEFT;
    }
    break;
  case FORWARD:
    break;
  case STOP:
    move_state = RIGHT;
    break;
  default:
    ROS_ERROR("Invalid state");
    break;
  }
}

// Right hand rule, for when all surrounding cells have same visit count
void RHRnextState(int32_t &move_state, bool bump) {
  switch (move_state) {
  case RIGHT:
    if (bump) { // T1.1
      move_state = LEFT;
    } else { // T1.4
      move_state = FORWARD;
    }
    break;
  case LEFT:
    if (bump) { // T1.2
      move_state = LEFT;
    } else { // T1.3
      move_state = FORWARD;
    }
    break;
  case FORWARD:
    break;
  case STOP:
    move_state = RIGHT;
    break;
  default:
    ROS_ERROR("Invalid state");
    break;
  }
}

// State Chart Top Level Function (state block)
void nextState(int32_t &move_state, bool bump, int32_t new_orientation,
               position currentPos, bool stopMove) {
  static std::map<int32_t, int32_t> surroundingPos;
  static int32_t minVisitDirection = NA;
  static int32_t currentState = SOLVING;

  switch (currentState) {
  case SOLVING: // S1
    move_state = STOP;
    surroundingPos = getSurroundingPos(currentPos);
    minVisitDirection = getMinVisitDirection(surroundingPos, new_orientation);
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
    RHRnextState(move_state, bump);
    if (move_state == FORWARD) { // T5
      currentState = SOLVING;
    }
    break;
  case LEASTVISIT: // S3
    leastVisitNextState(move_state, bump, new_orientation, minVisitDirection,
                        surroundingPos);
    if (move_state == FORWARD) { // T4
      currentState = SOLVING;
    }
    break;
  case SOLVED: // S4
    move_state = STOP;
    currentState = SOLVED; // T6
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
      4; // bigger number slows down simulation so you can see what's happening
  static const int32_t CYCLE_DECREASE = 1;
  static int32_t cycle = 0;
  static int32_t move_state = STOP;
  static position currentPos = {MAZE_CENTER, MAZE_CENTER};
  static int32_t new_orientation = NORTH;

  ROS_INFO("Turtle update Called  cycle=%d", cycle);
  if (cycle == 0) {
    nextState(move_state, bumped, new_orientation, currentPos, stopMove);

    ROS_INFO("Orientation=%d  STATE=%d", new_orientation, move_state);

    cycle = TIMEOUT;
    switch (move_state) {
    case FORWARD:
      if (new_orientation == SOUTH) {
        currentPos.y -= 1;
      }
      if (new_orientation == EAST) {
        currentPos.x += 1;
      }
      if (new_orientation == NORTH) {
        currentPos.y += 1;
      }
      if (new_orientation == WEST) {
        currentPos.x -= 1;
      }
      updateVisitsLocal(currentPos);
      return MOVE;
    case RIGHT:
      rotateDirection(new_orientation, true);
      return TURN_RIGHT;
    case LEFT:
      rotateDirection(new_orientation, false);
      return TURN_LEFT;
    case STOP:
      return NO_MOVE;
    default:
      ROS_ERROR("Invalid state");
      break;
    }
  }
  cycle -= CYCLE_DECREASE; // decrease cycle
  return NO_MOVE; // don't submit changes
}
