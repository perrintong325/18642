/*
 * 18-642 Unit Testing Example
 * Example code by Milda Zizyte
 * This code exercises Transitions T1 and T2 of the Project 8
 * dummy turtle statechart. It uses the CUnit framework (cunit.sourceforge.net)
 */

#include "student_mock.h"
#include <CUnit/Basic.h>

state moving_state;
std::map<direction, int32_t> surroundingPos;
direction minVisitDirection;
p7_state currentState;

// Test T1
void test_t1() {
  moving_state = RIGHT;
  surroundingPos = {{NORTH, 0}, {SOUTH, 0}, {EAST, 0}, {WEST, 0}};
  minVisitDirection = NA;
  currentState = SOLVING;
  nextState(moving_state, false, NORTH, false,
            surroundingPos, minVisitDirection, currentState);

  CU_ASSERT_EQUAL(currentState, RHR);
  CU_ASSERT_EQUAL(moving_state, STOP);
}

// Test T1.1
void test_t1_1() {
  moving_state = RIGHT;
  currentState = RHR;
  RHRnextState(moving_state, true);
  CU_ASSERT_EQUAL(currentState, RHR);
  CU_ASSERT_EQUAL(moving_state, LEFT);
}

// Test T1.2
void test_t1_2() {
  moving_state = LEFT;
  currentState = RHR;
  RHRnextState(moving_state, true);
  CU_ASSERT_EQUAL(currentState, RHR);
  CU_ASSERT_EQUAL(moving_state, LEFT);
}

// Test T1.3
void test_t1_3() {
  moving_state = LEFT;
  currentState = RHR;
  RHRnextState(moving_state, false);
  CU_ASSERT_EQUAL(currentState, RHR);
  CU_ASSERT_EQUAL(moving_state, FORWARD);
}

// Test T1.4
void test_t1_4() {
  moving_state = RIGHT;
  currentState = RHR;
  RHRnextState(moving_state, false);
  CU_ASSERT_EQUAL(currentState, RHR);
  CU_ASSERT_EQUAL(moving_state, FORWARD);
}

void test_RHR_default() {
  resetError();
  moving_state = WRONG;
  currentState = RHR;
  RHRnextState(moving_state, false);
  CU_ASSERT_EQUAL(currentState, RHR);
  CU_ASSERT_EQUAL(moving_state, 5);
  CU_ASSERT_EQUAL(checkError(), true);
}

// Test T2
void test_t2() {
  moving_state = RIGHT;
  surroundingPos = {{NORTH, 0}, {SOUTH, 0}, {EAST, 0}, {WEST, 0}};
  
  // T2 when minVisitDirection is NORTH
  minVisitDirection = NORTH;
  currentState = SOLVING;
  nextState(moving_state, false, NORTH, false,
            surroundingPos, minVisitDirection, currentState);

  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, STOP);

  // T2 when minVisitDirection is EAST
  minVisitDirection = EAST;
  currentState = SOLVING;
  nextState(moving_state, false, NORTH, false,
            surroundingPos, minVisitDirection, currentState);

  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, STOP);

  // T2 when minVisitDirection is SOUTH
  minVisitDirection = SOUTH;
  currentState = SOLVING;
  nextState(moving_state, false, NORTH, false,
            surroundingPos, minVisitDirection, currentState);

  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, STOP);

  // T2 when minVisitDirection is WEST
  minVisitDirection = WEST;
  currentState = SOLVING;
  nextState(moving_state, false, NORTH, false,
            surroundingPos, minVisitDirection, currentState);

  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, STOP);
}

// Test T2.1
void test_t2_1() {
  moving_state = RIGHT;
  surroundingPos = {{NORTH, 1}, {SOUTH, 1}, {EAST, 1}, {WEST, 0}};

  // T2.1 when minVisitDirection and orientation are NORTH
  minVisitDirection = NORTH;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, NORTH, minVisitDirection, surroundingPos);

  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, WEST);

  // T2.1 when minVisitDirection and orientation are EAST
  moving_state = RIGHT;
  minVisitDirection = EAST;
  leastVisitNextState(moving_state, true, EAST, minVisitDirection, surroundingPos);

  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, WEST);

  // T2.1 when minVisitDirection and orientation are SOUTH
  moving_state = RIGHT;
  minVisitDirection = SOUTH;
  leastVisitNextState(moving_state, true, SOUTH, minVisitDirection, surroundingPos);

  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, WEST);

  // T2.1 when minVisitDirection and orientation and orientation are WEST
  surroundingPos = {{NORTH, 1}, {SOUTH, 1}, {EAST, 0}, {WEST, 1}};
  moving_state = RIGHT;
  minVisitDirection = WEST;
  leastVisitNextState(moving_state, true, WEST, minVisitDirection, surroundingPos);

  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, EAST);
}

// Test T2.3
void test_t2_3() {
  moving_state = LEFT;
  surroundingPos = {{NORTH, 1}, {SOUTH, 1}, {EAST, 1}, {WEST, 0}};

  // T2.3 when minVisitDirection and orientation are NORTH
  minVisitDirection = NORTH;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, NORTH, minVisitDirection, surroundingPos);

  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, WEST);

  // T2.3 when minVisitDirection and orientation are EAST
  minVisitDirection = EAST;
  leastVisitNextState(moving_state, true, EAST, minVisitDirection, surroundingPos);

  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, WEST);

  // T2.3 when minVisitDirection and orientation are SOUTH
  minVisitDirection = SOUTH;
  leastVisitNextState(moving_state, true, SOUTH, minVisitDirection, surroundingPos);

  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, WEST);

  // T2.3 when minVisitDirection and orientation are WEST
  surroundingPos = {{NORTH, 1}, {SOUTH, 1}, {EAST, 0}, {WEST, 1}};
  minVisitDirection = WEST;
  leastVisitNextState(moving_state, true, WEST, minVisitDirection, surroundingPos);

  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, EAST);
}

void test_t2_4() {
  moving_state = RIGHT;
  surroundingPos = {{NORTH, 1}, {SOUTH, 1}, {EAST, 1}, {WEST, 0}};

  // T2.4 when minVisitDirection is EAST and orientation is NORTH
  moving_state = RIGHT;
  minVisitDirection = EAST;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, NORTH, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, EAST);

  // T2.4 when minVisitDirection is SOUTH and orientation is NORTH
  moving_state = RIGHT;
  minVisitDirection = SOUTH;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, NORTH, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, SOUTH);

  // T2.4 when minVisitDirection is WEST and orientation is NORTH
  moving_state = RIGHT;
  minVisitDirection = WEST;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, NORTH, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, WEST);

  // T2.4 when minVisitDirection is SOUTH and orientation is EAST
  moving_state = RIGHT;
  minVisitDirection = SOUTH;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, EAST, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, SOUTH);

  // T2.4 when minVisitDirection is WEST and orientation is EAST
  moving_state = RIGHT;
  minVisitDirection = WEST;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, EAST, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, WEST);

  // T2.4 when minVisitDirection is NORTH and orientation is EAST
  moving_state = RIGHT;
  minVisitDirection = NORTH;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, EAST, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, NORTH);

  // T2.4 when minVisitDirection is WEST and orientation is SOUTH
  moving_state = RIGHT;
  minVisitDirection = WEST;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, SOUTH, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, WEST);

  // T2.4 when minVisitDirection is NORTH and orientation is SOUTH
  moving_state = RIGHT;
  minVisitDirection = NORTH;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, SOUTH, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, NORTH);

  // T2.4 when minVisitDirection is EAST and orientation is SOUTH
  moving_state = RIGHT;
  minVisitDirection = EAST;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, SOUTH, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, EAST);

  // T2.4 when minVisitDirection is NORTH and orientation is WEST
  moving_state = RIGHT;
  minVisitDirection = NORTH;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, WEST, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, NORTH);

  // T2.4 when minVisitDirection is EAST and orientation is WEST
  moving_state = RIGHT;
  minVisitDirection = EAST;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, WEST, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, EAST);

  // T2.4 when minVisitDirection is SOUTH and orientation is WEST
  moving_state = RIGHT;
  minVisitDirection = SOUTH;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, WEST, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, SOUTH);
}

void test_t2_5() {
  moving_state = LEFT;
  surroundingPos = {{NORTH, 1}, {SOUTH, 1}, {EAST, 1}, {WEST, 0}};

  // T2.5 when minVisitDirection is EAST and orientation is NORTH
  minVisitDirection = EAST;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, NORTH, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, EAST);

  // T2.5 when minVisitDirection is SOUTH and orientation is NORTH
  minVisitDirection = SOUTH;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, NORTH, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, SOUTH);

  // T2.5 when minVisitDirection is WEST and orientation is NORTH
  minVisitDirection = WEST;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, NORTH, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, WEST);

  // T2.5 when minVisitDirection is SOUTH and orientation is EAST
  minVisitDirection = SOUTH;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, EAST, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, SOUTH);

  // T2.5 when minVisitDirection is WEST and orientation is EAST
  minVisitDirection = WEST;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, EAST, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, WEST);

  // T2.5 when minVisitDirection is NORTH and orientation is EAST
  minVisitDirection = NORTH;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, EAST, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, NORTH);

  // T2.5 when minVisitDirection is WEST and orientation is SOUTH
  minVisitDirection = WEST;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, SOUTH, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, WEST);

  // T2.5 when minVisitDirection is NORTH and orientation is SOUTH
  minVisitDirection = NORTH;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, SOUTH, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, NORTH);

  // T2.5 when minVisitDirection is EAST and orientation is SOUTH
  minVisitDirection = EAST;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, SOUTH, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, EAST);

  // T2.5 when minVisitDirection is NORTH and orientation is WEST
  minVisitDirection = NORTH;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, WEST, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, NORTH);

  // T2.5 when minVisitDirection is EAST and orientation is WEST
  minVisitDirection = EAST;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, WEST, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, EAST);

  // T2.5 when minVisitDirection is SOUTH and orientation is WEST
  moving_state = RIGHT;
  minVisitDirection = SOUTH;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, true, WEST, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, LEFT);
  CU_ASSERT_EQUAL(minVisitDirection, SOUTH);
}

void test_t2_6() {
  moving_state = LEFT;
  surroundingPos = {{NORTH, 1}, {SOUTH, 1}, {EAST, 1}, {WEST, 0}};

  // T2.6 when minVisitDirection and orientation are NORTH
  minVisitDirection = NORTH;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, false, NORTH, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, FORWARD);
  CU_ASSERT_EQUAL(minVisitDirection, NORTH);

  // T2.6 when minVisitDirection and orientation are EAST
  moving_state = LEFT;
  minVisitDirection = EAST;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, false, EAST, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, FORWARD);
  CU_ASSERT_EQUAL(minVisitDirection, EAST);

  // T2.6 when minVisitDirection and orientation are SOUTH
  moving_state = LEFT;
  minVisitDirection = SOUTH;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, false, SOUTH, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, FORWARD);
  CU_ASSERT_EQUAL(minVisitDirection, SOUTH);

  // T2.6 when minVisitDirection and orientation are WEST
  moving_state = LEFT;
  minVisitDirection = WEST;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, false, WEST, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, FORWARD);
  CU_ASSERT_EQUAL(minVisitDirection, WEST);
}

void test_t2_7() {
  moving_state = RIGHT;
  surroundingPos = {{NORTH, 1}, {SOUTH, 1}, {EAST, 1}, {WEST, 0}};

  // T2.7 when minVisitDirection and orientation are NORTH
  minVisitDirection = NORTH;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, false, NORTH, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, FORWARD);
  CU_ASSERT_EQUAL(minVisitDirection, NORTH);

  // T2.7 when minVisitDirection and orientation are EAST
  moving_state = RIGHT;
  minVisitDirection = EAST;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, false, EAST, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, FORWARD);
  CU_ASSERT_EQUAL(minVisitDirection, EAST);

  // T2.7 when minVisitDirection and orientation are SOUTH
  moving_state = RIGHT;
  minVisitDirection = SOUTH;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, false, SOUTH, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, FORWARD);
  CU_ASSERT_EQUAL(minVisitDirection, SOUTH);

  // T2.7 when minVisitDirection and orientation are WEST
  moving_state = RIGHT;
  minVisitDirection = WEST;
  currentState = LEASTVISIT;
  leastVisitNextState(moving_state, false, WEST, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(currentState, LEASTVISIT);
  CU_ASSERT_EQUAL(moving_state, FORWARD);
  CU_ASSERT_EQUAL(minVisitDirection, WEST);
}

// Test invalid least visited state
void test_LeastVisited_default() {
  resetError();
  moving_state = WRONG;
  leastVisitNextState(moving_state, false, WEST, minVisitDirection, surroundingPos);
  CU_ASSERT_EQUAL(checkError(), true);
}

// Test T3
void test_t3() {
  moving_state = RIGHT;
  surroundingPos = {{NORTH, 0}, {SOUTH, 0}, {EAST, 0}, {WEST, 0}};
  minVisitDirection = NA;
  currentState = SOLVING;
  nextState(moving_state, false, NORTH, true,
            surroundingPos, minVisitDirection, currentState);

  CU_ASSERT_EQUAL(currentState, SOLVED);
  CU_ASSERT_EQUAL(moving_state, STOP);
}

// Test T4
void test_t4() {
  moving_state = FORWARD;
  surroundingPos = {{NORTH, 0}, {SOUTH, 0}, {EAST, 0}, {WEST, 0}};
  minVisitDirection = NORTH;
  currentState = LEASTVISIT;
  nextState(moving_state, false, NORTH, false,
            surroundingPos, minVisitDirection, currentState);

  CU_ASSERT_EQUAL(currentState, SOLVING);
  CU_ASSERT_EQUAL(moving_state, FORWARD);
}

// Test T5
void test_t5() {
  moving_state = RIGHT;
  surroundingPos = {{NORTH, 0}, {SOUTH, 0}, {EAST, 0}, {WEST, 0}};
  minVisitDirection = NORTH;
  currentState = RHR;
  nextState(moving_state, false, NORTH, false,
            surroundingPos, minVisitDirection, currentState);

  CU_ASSERT_EQUAL(currentState, SOLVING);
  CU_ASSERT_EQUAL(moving_state, FORWARD);
}

// Test T6
void test_t6() {
  moving_state = RIGHT;
  surroundingPos = {{NORTH, 0}, {SOUTH, 0}, {EAST, 0}, {WEST, 0}};
  minVisitDirection = NA;
  currentState = SOLVED;
  nextState(moving_state, true, NORTH, false,
            surroundingPos, minVisitDirection, currentState);

  CU_ASSERT_EQUAL(currentState, SOLVED);
  CU_ASSERT_EQUAL(moving_state, STOP);
}

int init() {
  moving_state = RIGHT;
  surroundingPos = {{NORTH, 0}, {SOUTH, 0}, {EAST, 0}, {WEST, 0}};
  minVisitDirection = NA;
  currentState = SOLVING;
  return 0;
}

int cleanup() {
  // Any test cleanup code goes here
  return 0;
}

/* Skeleton code from http://cunit.sourceforge.net/example.html */
int main() {

  CU_pSuite pSuite1 = NULL;
  CU_pSuite pSuite2 = NULL;
  CU_pSuite pSuite3 = NULL;

  /* initialize the CUnit test registry */
  if (CUE_SUCCESS != CU_initialize_registry())
    return CU_get_error();

  /* add a suite to the registry */
  pSuite1 = CU_add_suite("Suite_1 - Overall States", init, cleanup);
  pSuite2 = CU_add_suite("Suite_2 - RHR Sub-states", init, cleanup);
  pSuite3 = CU_add_suite("Suite_3 - LeastVisited Sub-states", init, cleanup);
  if (NULL == pSuite1 || NULL == pSuite2 || NULL == pSuite3) {
    CU_cleanup_registry();
    return CU_get_error();
  }

  /* Suite 1 with overall states*/
  if ((NULL == CU_add_test(pSuite1, "test of transition T1", test_t1)) ||
      (NULL == CU_add_test(pSuite2, "test of transition T1.1", test_t1_1)) ||
      (NULL == CU_add_test(pSuite2, "test of transition T1.2", test_t1_2)) ||
      (NULL == CU_add_test(pSuite2, "test of transition T1.3", test_t1_3)) ||
      (NULL == CU_add_test(pSuite2, "test of transition T1.4", test_t1_4)) ||
      (NULL == CU_add_test(pSuite2, "test of invalid state", test_RHR_default)) ||
      (NULL == CU_add_test(pSuite1, "test of transition T2", test_t2)) ||
      (NULL == CU_add_test(pSuite3, "test of transition T2.1", test_t2_1)) ||
      (NULL == CU_add_test(pSuite3, "test of transition T2.3", test_t2_3)) ||
      (NULL == CU_add_test(pSuite3, "test of transition T2.4", test_t2_4)) ||
      (NULL == CU_add_test(pSuite3, "test of transition T2.5", test_t2_5)) ||
      (NULL == CU_add_test(pSuite3, "test of transition T2.6", test_t2_6)) ||
      (NULL == CU_add_test(pSuite3, "test of transition T2.7", test_t2_7)) ||
      (NULL == CU_add_test(pSuite3, "test of invalid state", test_LeastVisited_default)) ||
      (NULL == CU_add_test(pSuite1, "test of transition T3", test_t3)) ||
      (NULL == CU_add_test(pSuite1, "test of transition T4", test_t4)) ||
      (NULL == CU_add_test(pSuite1, "test of transition T5", test_t5)) ||
      (NULL == CU_add_test(pSuite1, "test of transition T6", test_t6))) {
    CU_cleanup_registry();
    return CU_get_error();
  }


  /* Run all tests using the CUnit Basic interface */
  CU_basic_set_mode(CU_BRM_VERBOSE);
  CU_basic_run_tests();
  CU_cleanup_registry();
  return CU_get_error();

  return 0;
}