#include "monitor_interface.h"

// Circular queue to store recent bumpInterrupt calls
const int MAX_BUMPS = 8;
Endpoints bumpQueue[MAX_BUMPS];
int bumpIndex = 0;
bool bumpQueueFull = false;

// Variables to track the turtle's state
int last_x = -1;
int last_y = -1;
Orientation last_orientation = EAST;
bool orientation_initialized = false;

// Function to add a bumpInterrupt call to the queue
void addBump(int x1, int y1, int x2, int y2, bool bumped) {
    bumpQueue[bumpIndex] = {x1, y1, x2, y2, bumped};
    bumpIndex = (bumpIndex + 1) % MAX_BUMPS;
    if (bumpIndex == 0) {
        bumpQueueFull = true;
    }
}

// Function to check if a wall segment is in the bumpQueue
bool bumpedInMemory(Endpoints wall) {
    for (int i = 0; i < (bumpQueueFull ? MAX_BUMPS : bumpIndex); i++) {
        if ((bumpQueue[i].x1 == wall.x1 && bumpQueue[i].y1 == wall.y1 && bumpQueue[i].x2 == wall.x2 && bumpQueue[i].y2 == wall.y2) ||
            (bumpQueue[i].x1 == wall.x2 && bumpQueue[i].y1 == wall.y2 && bumpQueue[i].x2 == wall.x1 && bumpQueue[i].y2 == wall.y1)) {
            if (bumpQueue[i].bumped) {
                ROS_WARN("VIOLATION: Turtle crossing a wall segment");
            }
            return true;
        }
    }
    return false;
}

// Function to clear the bump queue
void clearBumpQueue() {
    bumpIndex = 0;
    bumpQueueFull = false;
}

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    if (orientation_initialized) {
        // Check if the turtle has moved
        if (x != last_x || y != last_y) {
            Endpoints wall = {0, 0, 0, 0};
            switch(last_orientation) {
                case EAST:
                    wall = {last_x + 1, last_y, last_x + 1, last_y + 1};
                    break;
                case WEST:
                    wall = {last_x, last_y + 1, last_x, last_y};
                    break;
                case NORTH:
                    wall = {last_x, last_y, last_x + 1, last_y};
                    break;
                case SOUTH:
                    wall = {last_x + 1, last_y + 1, last_x, last_y + 1};
                    break;
            }
            // Check if the turtle has checked the wall segment before crossing it
            if (!bumpedInMemory(wall)) {
                ROS_WARN("VIOLATION: Turtle crossed a wall segment without checking it!");
            }
            clearBumpQueue();
        }
    }

    // Update the last known position
    last_x = x;
    last_y = y;
    last_orientation = o;
    orientation_initialized = true;
}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
    // Add the bumpInterrupt call to the queue
    addBump(x1, y1, x2, y2, bumped);
}

/*
 * Empty interrupt handlers beyond this point
 */

void visitInterrupt(ros::Time t, int visits) {
}

void tickInterrupt(ros::Time t) {
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
}

void printStartMsg() {
    std::time_t now = std::time(nullptr);
    std::cerr << "Monitor yiuchunt_wall_monitor is running at " << std::ctime(&now);
}