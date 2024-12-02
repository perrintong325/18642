#include "monitor_interface.h"

// Flags to track if each interrupt has been called
bool pose_called = false;
bool visit_called = false;
bool bump_called = false;

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    if (pose_called) {
        ROS_WARN("VIOLATION: poseInterrupt called more than once between tickInterrupts!");
    }
    pose_called = true;
}

void visitInterrupt(ros::Time t, int visits) {
    if (visit_called) {
        ROS_WARN("VIOLATION: visitInterrupt called more than once between tickInterrupts!");
    }
    visit_called = true;
}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
    if (bump_called) {
        ROS_WARN("VIOLATION: bumpInterrupt called more than once between tickInterrupts!");
    }
    bump_called = true;
}

void tickInterrupt(ros::Time t) {
    // Reset flags
    pose_called = false;
    visit_called = false;
    bump_called = false;
}

/*
 * Empty interrupt handlers beyond this point
 */
void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
}

void printStartMsg() {
    std::time_t now = std::time(nullptr);
    std::cerr << "Monitor yiuchunt_tick_monitor is running at " << std::ctime(&now);
}