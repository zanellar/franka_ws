#ifdef NDEBUG
#undef NDEBUG
#endif
#include <franka_example_controllers/directional_hardware_qp.h>
#include <cassert>
#include <iostream>
#include <random>
using namespace franka_example_controllers::hardware_cbf;
int main() {
  std::array<JointLimits,7> limits;
  for(auto& l:limits) l={-2,2,2,10,10,5};
  Seven zero{}, lower{}, upper{}, a{}, nominal{}, solution{};
  lower.fill(-1); upper.fill(1); a[0]=1;
  assert(project(zero,lower,upper,a,.5,solution));
  assert(std::abs(solution[0]-.5)<1e-12);
  assert(!project(zero,lower,upper,a,2,solution));
  a.fill(0); assert(project(zero,lower,upper,a,0,solution));
  assert(!project(zero,lower,upper,a,.001,solution));
  a[0]=1e-12; assert(project(zero,lower,upper,a,5e-13,solution));
  assert(std::abs(solution[0]-.5)<1e-12);
  auto e=envelope(limits,zero,zero,zero,100,999);
  assert(e.valid && e.gain>0 && e.gain<1);
  assert(std::abs(predict(e,e.upper)[0]-.999)<1e-12);
  e=envelope(limits,zero,zero,zero,1000,500);
  assert(e.valid && e.gain==1 && e.upper[0]==.5);
  Seven q=zero,dq=zero,prev=zero;
  q[0]=1.99; dq[0]=1;
  e=envelope(limits,q,dq,prev,100,999);
  assert(!e.valid); // soft limit requires braking beyond this cycle's rate envelope
  assert(e.soft_upper[0]<0 && e.soft_upper[6]==10); // all fallback bounds populated
  dq[0]=0; e=envelope(limits,q,dq,prev,100,999);
  assert(e.valid && std::abs(e.soft_upper[0]-.5)<1e-12);
  assert(!envelope(limits,q,dq,prev,100,1001).valid);
  // Feasibility and first-order optimality against independently sampled feasible points.
  std::mt19937 rng(12345);
  std::uniform_real_distribution<double> uniform(-1,1);
  for(int sample=0;sample<400;++sample) {
    double maximum=0;
    for(int i=0;i<7;++i) { a[i]=uniform(rng); nominal[i]=2*uniform(rng); maximum+=std::abs(a[i]); }
    const double rhs=.6*maximum;
    assert(project(nominal,lower,upper,a,rhs,solution));
    double value=0; for(int i=0;i<7;++i) { assert(solution[i]>=-1 && solution[i]<=1); value+=a[i]*solution[i]; }
    assert(value>=rhs-1e-12);
    for(int point=0;point<80;++point) {
      Seven candidate{}; double attained=0,variational=0;
      for(int i=0;i<7;++i) { candidate[i]=uniform(rng); attained+=a[i]*candidate[i];
        variational+=(solution[i]-nominal[i])*(candidate[i]-solution[i]); }
      if(attained>=rhs) assert(variational>=-1e-10);
    }
    assert(!project(nominal,lower,upper,a,maximum+.001,solution));
  }
  // Filtered physical CBF, using an inverse-transformed halfspace and soft/rate bounds.
  e=envelope(limits,zero,zero,zero,100,999);
  a.fill(0); a[0]=e.gain;
  assert(project(zero,e.lower,e.upper,a,.7,solution));
  const auto physical=predict(e,solution);
  assert(physical[0]>=.7-1e-12 && physical[0]<=.999+1e-12);
  std::cout << "Hardware QP, filter, soft-limit and rate-limit tests passed\n";
}
