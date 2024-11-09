/*
 * 18-642 Unit Testing Example
 * Example code by Milda Zizyte
 */

#include "student_mock.h"
#include <iostream>
#include <cstdarg> // For va_list, va_start, va_end
#include <cstdio>  // For vsnprintf

static orientation mock_orientation;
static bool mock_bump;
static bool mock_error = false;

/* Functions called by dummy_turtle */
void setOrientation(orientation ornt) {
  mock_orientation = ornt;
}

bool will_bump() {
  return mock_bump;
}

/* Functions used to instrument CUnit tests */

void displayVisits(int visits) {
  return;
}

bool checkError() {
  return mock_error;
}

void resetError() {
  mock_error = false;
}

/* Dummy ROS_ERROR */
void ROS_ERROR(std::string e) {
  mock_error = true;
  // std::cout << e << std::endl;
}

void ROS_INFO(const char* format, ...) {
    const int BUFFER_SIZE = 1024;
    char buffer[BUFFER_SIZE];

    va_list args;
    va_start(args, format);
    vsnprintf(buffer, BUFFER_SIZE, format, args);
    va_end(args);

    std::cout << buffer << std::endl;
}
