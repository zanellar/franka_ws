#ifdef NDEBUG
#undef NDEBUG
#endif
#include <franka_trajectory/initialization_policy.h>
#include <cassert>
#include <functional>
#include <iostream>
#include <limits>
#include <vector>

using namespace franka_trajectory;
void throws(const std::function<void()>& action) {
  bool rejected=false;
  try { action(); } catch (const std::runtime_error&) { rejected=true; }
  assert(rejected);
}

struct FakeOperations {
  int failure=-1;
  bool reference_ready=true;
  bool cartesian=true, joint=false;
  bool prepared=false, synchronized=false;
  std::vector<int> calls;
  void step(int id) {
    calls.push_back(id);
    if (id==failure) throw std::runtime_error("injected operation failure");
  }
  void preflight() { step(0); }
  void invalidateReference() { reference_ready=false; step(1); }
  void acquireJointController() { step(2); cartesian=false; joint=true; }
  void moveAndSettle() { assert(joint && !cartesian); step(3); }
  void prepareCartesian() { assert(joint && !cartesian); step(4); prepared=true; }
  void resumeCartesian() { assert(prepared); step(5); cartesian=true; joint=false; }
  void synchronizeReference() {
    assert(cartesian && !joint && prepared);
    step(6); synchronized=true; reference_ready=true;
  }
};

int main() {
  const std::array<JointBounds,7> bounds{{
    {-2.3093,2.3093,2.0}, {-1.5133,1.5133,2.0}, {-2.4937,2.4937,2.0},
    {-2.7478,-.4461,2.0}, {-2.48,2.48,2.0}, {.8521,4.2094,2.0}, {-2.6895,2.6895,2.0}}};
  const std::array<double,7> home{{0,-.785398163,0,-2.35619449,0,1.57079632679,.785398163397}};
  validateJointTarget(home,bounds,.02,6.0);
  for (size_t i=0; i<7; ++i) {
    auto q=home;
    q[i]=bounds[i].lower;
    throws([&] { validateJointTarget(q,bounds,.02,6.0); });
    q[i]=bounds[i].upper;
    throws([&] { validateJointTarget(q,bounds,.02,6.0); });
    q[i]=std::numeric_limits<double>::quiet_NaN();
    throws([&] { validateJointTarget(q,bounds,.02,6.0); });
  }
  throws([&] { validateJointTarget(home,bounds,.02,0); });
  throws([&] { validateJointTarget(home,bounds,.02,std::numeric_limits<double>::infinity()); });
  throws([&] { validateJointTarget(home,bounds,-.01,6); });
  auto target=home; target[0]=1.0;
  assert(std::abs(minimumJointDuration(home,target,bounds,.2,.5)-4.6875)<1e-12);
  assert(minimumJointDuration(home,home,bounds,.2,.5)==2.0);
  throws([&] { minimumJointDuration(home,target,bounds,0,.5); });
  throws([&] { minimumJointDuration(home,target,bounds,.2,0); });
  const double hardware_duration=hardwareJointDuration(home,target,.5,1.0);
  assert(hardware_duration>=std::sqrt((10/std::sqrt(3.0))/.5));
  assert(hardware_duration>=std::cbrt(60.0));
  throws([&] { hardwareJointDuration(home,target,0,1); });
  throws([&] { hardwareJointDuration(home,target,.5,0); });
  std::array<double,7> dq{};
  assert(atRest(dq,.02));
  dq[5]=-.03; assert(!atRest(dq,.02));
  dq[5]=std::numeric_limits<double>::quiet_NaN(); assert(!atRest(dq,.02));

  double x=.3,y=.1,z=.6;
  addCartesianDisplacement(x,y,z,.01,-.02,.03);
  assert(std::abs(x-.31)<1e-12 && std::abs(y-.08)<1e-12 && std::abs(z-.63)<1e-12);
  const double old_x=x, old_y=y, old_z=z;
  throws([&] { addCartesianDisplacement(x,y,z,1,std::numeric_limits<double>::infinity(),0); });
  assert(x==old_x && y==old_y && z==old_z);  // Atomic rejection, no partial x update.
  x=std::numeric_limits<double>::max();
  throws([&] { addCartesianDisplacement(x,y,z,x,0,0); });

  FakeOperations success;
  performJointInitialization(success);
  assert(success.synchronized && success.reference_ready && success.cartesian && !success.joint);
  for (int failed=0; failed<=6; ++failed) {
    FakeOperations op; op.failure=failed;
    throws([&] { performJointInitialization(op); });
    assert(op.calls.size()==static_cast<size_t>(failed+1));
    assert(!op.synchronized);
    assert(op.reference_ready==(failed==0));
    if (failed>=3 && failed<=5) assert(op.joint && !op.cartesian);
    if (failed==6) assert(op.cartesian && !op.reference_ready);
  }
  // Failure keeps commands blocked; a later explicit successful retry can recover.
  FakeOperations retry; retry.failure=3;
  throws([&] { performJointInitialization(retry); });
  retry.failure=-1; retry.calls.clear();
  performJointInitialization(retry);
  assert(retry.reference_ready && retry.synchronized);
  std::cout << "Initialization policy and XYZ tests passed\n";
}
