#include "monitor_interface.h"

// Variables to track the turtle's state
int last_x = -1;
int last_y = -1;
Orientation last_orientation = EAST;
bool orientation_initialized = false;

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    // Update the last known position and orientation
    last_x = x;
    last_y = y;
    last_orientation = o;
    orientation_initialized = true;
}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
    if (orientation_initialized) {
        // Compute the valid wall segment based on the current position and orientation
        int valid_x1 = last_x, valid_y1 = last_y, valid_x2 = last_x, valid_y2 = last_y;
        switch (last_orientation) {
            case NORTH:
                valid_x2 += 1;
                break;
            case EAST:
                valid_x1 += 1;
                valid_x2 += 1;
                valid_y2 += 1;
                break;
            case SOUTH:
                valid_x1 += 1;
                valid_y1 += 1;
                valid_y2 += 1;
                break;
            case WEST:
                valid_y1 += 1;
                break;
            default:
                ROS_ERROR("Invalid orientation");
                return;
        }

        // Check if the turtle is facing the wall segment it is calling bumped for
        if (!((x1 == valid_x1 && y1 == valid_y1 && x2 == valid_x2 && y2 == valid_y2) ||
              (x1 == valid_x2 && y1 == valid_y2 && x2 == valid_x1 && y2 == valid_y1))) {
            ROS_WARN("VIOLATION: Turtle called bumped for a wall segment it is not facing!");
        }
    }
}

/*
 * Empty interrupt handlers beyond this point
 */

void tickInterrupt(ros::Time t) {
}

void visitInterrupt(ros::Time t, int visits) {
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
}