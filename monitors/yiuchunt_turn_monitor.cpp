#include "monitor_interface.h"

// Keeps track of the last orientation received
static Orientation last_orientation;
static bool orientation_initialized = false;

/*
 * Whenever the turtle's orientation changes, compare the current orientation
 * to the previous orientation and throw an invariant violation if the change
 * exceeds 90 degrees.
 */
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
  const int wrapAround = 4;
  const int threshold = 2;
  const int difference = 1;
  if (orientation_initialized) {
    int change = abs(o - last_orientation);
    if (change > threshold) {
      change = wrapAround - change; // Adjust for wrap-around
    }
    if (change > difference) {
      ROS_WARN("VIOLATION: Orientation changed by more than 90 degrees from %d to %d!", last_orientation, o);
    }
  }

  // Update the last orientation
  last_orientation = o;
  orientation_initialized = true;
}

/*
 * Empty interrupt handlers beyond this point
 */

void tickInterrupt(ros::Time t) {
}

void visitInterrupt(ros::Time t, int visits) {
}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
}