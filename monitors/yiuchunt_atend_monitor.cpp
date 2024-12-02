#include "monitor_interface.h"

// Variables to track the turtle's state
int last_x = -1;
int last_y = -1;
bool orientation_initialized = false;
bool at_end_called = false;

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    // Update the last known position
    last_x = x;
    last_y = y;
    orientation_initialized = true;
}

void tickInterrupt(ros::Time t) {
    // Output "Successful atEnd" message to flush the ROS warning stream
    if (at_end_called) {
        ROS_INFO("Successful atEnd");
    }
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
    if (atEnd) {
        if (orientation_initialized && x == last_x && y == last_y) {
            at_end_called = true;
            ROS_INFO("Successful atEnd");
        } else {
            ROS_WARN("VIOLATION: Turtle called atEnd for a position it is not at!");
        }
    }
}

/*
 * Empty interrupt handlers beyond this point
 */

void visitInterrupt(ros::Time t, int visits) {
}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
}

void printStartMsg() {
    std::time_t now = std::time(nullptr);
    std::cerr << "Monitor yiuchunt_atend_monitor is running at " << std::ctime(&now);
}