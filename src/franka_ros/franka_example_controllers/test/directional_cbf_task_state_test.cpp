#include <franka_example_controllers/directional_cbf_task_state.h>
#include <cstdlib>
#include <iostream>
#include <limits>

using franka_example_controllers::DirectionalCbfTaskState;

void check(bool condition, const char* message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

int main() {
  for (unsigned char status : {3, 4, 5, 6}) {
    DirectionalCbfTaskState task;
    check(!task.aborted(), "Fresh controller must not be aborted");
    task.fail(status);
    for (int tick = 0; tick < 100; ++tick) {
      check(!task.consumeStartRequest(), "A control tick must not restart an aborted task");
      check(task.aborted() && task.error() == status, "Failure must remain latched");
    }
    task.fail(5);
    check(task.error() == status, "Diagnostics must preserve the original failure");
    task.requestStart();
    check(task.aborted(), "Callback must not clear the latch before the update cycle");
    check(task.consumeStartRequest(), "Explicit command must trigger one restart");
    check(!task.aborted(), "Explicit restart must clear the latch");
    check(!task.consumeStartRequest(), "Restart command must be consumed exactly once");
    task.fail(4);  // Failure of QP reinitialization during the new attempt.
    check(task.aborted(), "Failed reinitialization must abort again");
    task.requestStart();
    check(task.consumeStartRequest() && !task.aborted(), "A later attempt must remain possible");
  }
  const double tolerance = 1.e-5;
  check(DirectionalCbfTaskState::acceptsSolution(true, 0., tolerance), "Boundary must pass");
  check(DirectionalCbfTaskState::acceptsSolution(true, -tolerance, tolerance), "Tolerance boundary");
  check(!DirectionalCbfTaskState::acceptsSolution(true, -2*tolerance, tolerance), "Violation must abort");
  check(!DirectionalCbfTaskState::acceptsSolution(false, 1., tolerance), "Nonoptimal status must abort");
  check(!DirectionalCbfTaskState::acceptsSolution(true,
      std::numeric_limits<double>::quiet_NaN(), tolerance), "NaN must abort");
  check(!DirectionalCbfTaskState::acceptsSolution(true,
      std::numeric_limits<double>::infinity(), tolerance), "Infinite residual must abort");
  check(!DirectionalCbfTaskState::acceptsSolution(true, 0., -tolerance), "Invalid tolerance must fail");
  std::cout << "PASS: abort latch, explicit retry, failed retry, residual acceptance\n";
}
