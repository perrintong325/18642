#include "monitor_interface.h"

// Variables to track the turtle's state
int last_x = -1;
int last_y = -1;
Orientation last_orientation = EAST;
bool orientation_initialized = false;
bool moved = false;
bool turned = false;

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    if (orientation_initialized) {
        // Check if the turtle has turned
        if (o != last_orientation) {
            turned = true;
        }
        // Check if the turtle has moved
        if (x != last_x || y != last_y) {
            moved = true;
            // Ensure the turtle is facing the direction it is moving
            if ((x == last_x + 1 && o != EAST) ||
                (x == last_x - 1 && o != WEST) ||
                (y == last_y - 1 && o != NORTH) ||
                (y == last_y + 1 && o != SOUTH)) {
                ROS_WARN("VIOLATION: Turtle moved in a direction it was not facing!");
            }
        }
    }

    // Update the last known position and orientation
    last_x = x;
    last_y = y;
    last_orientation = o;
    orientation_initialized = true;
}

void tickInterrupt(ros::Time t) {
    // Check if the turtle moved and turned in the same round
    if (moved && turned) {
        ROS_WARN("VIOLATION: Turtle moved and turned in the same round!");
    }

    // Reset flags for the next round
    moved = false;
    turned = false;
}

/*
 * Empty interrupt handlers beyond this point
 */

void visitInterrupt(ros::Time t, int visits) {
}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
}