#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace franka_example_controllers {
namespace hardware_cbf {
using Seven = std::array<double, 7>;
constexpr double sample_time = 0.001;  // libfranka 0.13.3 control_loop.cpp
constexpr double library_torque_rate = 999.999;
inline double clip(double x, double lo, double hi) { return std::max(lo, std::min(x, hi)); }
struct JointLimits { double lower, upper, velocity, effort, k_position, k_velocity; };
struct Envelope {
  Seven lower{}, upper{}, soft_lower{}, soft_upper{}, previous{};
  double gain{1.0};
  bool valid{false};
};
inline double filterGain(double cutoff) {
  return cutoff >= 1000.0 ? 1.0 : sample_time / (sample_time + 1.0/(2.0*3.141592653589793*cutoff));
}
inline Envelope envelope(const std::array<JointLimits, 7>& limits, const Seven& q,
                         const Seven& dq, const Seven& previous, double cutoff, double rate) {
  Envelope e;
  if (!std::isfinite(cutoff) || cutoff <= 0 || !std::isfinite(rate) ||
      rate <= 0 || rate > library_torque_rate) return e;
  e.gain = filterGain(cutoff); e.previous = previous;
  bool feasible=true;
  for (size_t i=0; i<7; ++i) {
    const auto& l=limits[i];
    if (!std::isfinite(q[i]) || !std::isfinite(dq[i]) || !std::isfinite(previous[i]) ||
        !std::isfinite(l.lower) || !std::isfinite(l.upper) || l.lower>=l.upper ||
        !std::isfinite(l.velocity) || l.velocity<=0 || !std::isfinite(l.effort) || l.effort<=0 ||
        !std::isfinite(l.k_position) || l.k_position<0 ||
        !std::isfinite(l.k_velocity) || l.k_velocity<0) return e;
    // Same measured-state EffortJointSoftLimitsHandle used by FrankaHW.
    const double vmin=clip(-l.k_position*(q[i]-l.lower),-l.velocity,l.velocity);
    const double vmax=clip(-l.k_position*(q[i]-l.upper),-l.velocity,l.velocity);
    e.soft_lower[i]=clip(-l.k_velocity*(dq[i]-vmin),-l.effort,l.effort);
    e.soft_upper[i]=clip(-l.k_velocity*(dq[i]-vmax),-l.effort,l.effort);
    // Constrain the filtered command's rate by intersecting in RAW command space.
    e.lower[i]=std::max(e.soft_lower[i],previous[i]-rate*sample_time/e.gain);
    e.upper[i]=std::min(e.soft_upper[i],previous[i]+rate*sample_time/e.gain);
    if (e.lower[i]>e.upper[i]) feasible=false;
  }
  e.valid=feasible; return e;
}
inline Seven predict(const Envelope& e, const Seven& raw) {
  Seven out{};
  for (size_t i=0; i<7; ++i) {
    const double filtered=e.gain*clip(raw[i],e.soft_lower[i],e.soft_upper[i])+
                          (1.0-e.gain)*e.previous[i];
    out[i]=e.previous[i]+clip(filtered-e.previous[i],
                            -library_torque_rate*sample_time,library_torque_rate*sample_time);
  }
  return out;
}
// Diagnostic only: reason codes do not change any solver predicate/tolerance.
enum class ProjectionReason : unsigned char {
  None=0, NonfiniteInput=1, InvalidBox=2, ZeroNormalInfeasible=3,
  BoxInfeasible=4, NumericalFailure=5, InvalidEnvelope=6
};
struct ProjectionReport {
  ProjectionReason reason{ProjectionReason::None};
  double maximum_lhs{std::numeric_limits<double>::quiet_NaN()};
  double maximum_residual{std::numeric_limits<double>::quiet_NaN()};
};
// Exact strictly convex QP: min ||x-nominal||^2, lo<=x<=hi, a*x>=rhs.
// KKT: x(lambda)=clip(nominal+lambda*a,lo,hi). Fixed-size, allocation-free,
// at most 80 bisections; no OSQP setup/allocation inside the hardware callback.
inline bool project(const Seven& nominal,const Seven& lo,const Seven& hi,
                    const Seven& a,double rhs,Seven& solution, ProjectionReport* report=nullptr) {
  if (report) *report=ProjectionReport{};
  const auto fail=[report](ProjectionReason reason) {
    if (report) report->reason=reason;
    return false;
  };
  if (!std::isfinite(rhs)) return fail(ProjectionReason::NonfiniteInput);
  double scale=0.0;
  for(size_t i=0;i<7;++i) {
    if(!std::isfinite(nominal[i]) || !std::isfinite(lo[i]) || !std::isfinite(hi[i]) ||
       !std::isfinite(a[i])) return fail(ProjectionReason::NonfiniteInput);
    if (lo[i]>hi[i]) return fail(ProjectionReason::InvalidBox);
    solution[i]=clip(nominal[i],lo[i],hi[i]); scale=std::max(scale,std::abs(a[i]));
  }
  if(scale==0.0) {
    if (report) { report->maximum_lhs=0.0; report->maximum_residual=-rhs; }
    if (rhs<=0.0) return true;
    return fail(ProjectionReason::ZeroNormalInfeasible);
  }
  Seven n{}; double r=rhs/scale, value=0, maximum=0, high=0;
  if(!std::isfinite(r)) return fail(ProjectionReason::NumericalFailure);
  for(size_t i=0;i<7;++i) {
    n[i]=a[i]/scale; value+=n[i]*solution[i];
    const double endpoint=n[i]>=0 ? hi[i] : lo[i]; maximum+=n[i]*endpoint;
    if(n[i]!=0) high=std::max(high,(endpoint-nominal[i])/n[i]);
  }
  if (report) {
    report->maximum_lhs=maximum*scale;
    report->maximum_residual=(maximum-r)*scale;
  }
  if(value>=r) return true;
  if(maximum<r) return fail(ProjectionReason::BoxInfeasible);
  if(!std::isfinite(high)) return fail(ProjectionReason::NumericalFailure);
  double low=0;
  for(int iteration=0;iteration<80;++iteration) {
    const double middle=low+(high-low)*0.5; double achieved=0;
    for(size_t i=0;i<7;++i) achieved+=n[i]*clip(nominal[i]+middle*n[i],lo[i],hi[i]);
    if(achieved>=r) high=middle; else low=middle;
  }
  for(size_t i=0;i<7;++i) solution[i]=clip(nominal[i]+high*n[i],lo[i],hi[i]);
  return true;  // Controller rechecks the ORIGINAL physical residual/tolerance.
}
// Weighted projection: min sum((x_i-nominal_i)^2 / inverse_weights_i).
// Whitening is a change of coordinates, not a post-QP filter. Thus the same
// box and halfspace are enforced. Positive weights are mandatory.
inline bool projectWeighted(const Seven& nominal,const Seven& lo,const Seven& hi,
                            const Seven& a,double rhs,const Seven& inverse_weights,
                            Seven& solution,ProjectionReport* report=nullptr) {
  Seven root{},n{},l{},u{},coeff{},y{};
  bool identity=true;
  for (size_t i=0;i<7;++i) {
    const double w=inverse_weights[i];
    if (!std::isfinite(w) || w<=0) {
      if (report) { *report=ProjectionReport{}; report->reason=ProjectionReason::NonfiniteInput; }
      return false;
    }
    identity=identity && w==1.0;
    root[i]=std::sqrt(w);
    n[i]=nominal[i]/root[i]; l[i]=lo[i]/root[i]; u[i]=hi[i]/root[i];
    coeff[i]=a[i]*root[i];
  }
  if (identity) return project(nominal,lo,hi,a,rhs,solution,report);
  if (!project(n,l,u,coeff,rhs,y,report)) return false;
  for (size_t i=0;i<7;++i) solution[i]=clip(root[i]*y[i],lo[i],hi[i]);
  return true;  // Physical residual is still checked by the controller.
}
}  // namespace hardware_cbf
}  // namespace franka_example_controllers
