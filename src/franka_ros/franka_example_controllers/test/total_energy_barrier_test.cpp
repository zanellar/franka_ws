#include <franka_example_controllers/total_energy_barrier.h>
#include <franka_example_controllers/directional_hardware_qp.h>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace franka_example_controllers;
void require(bool ok, const char* why) { if (!ok) throw std::runtime_error(why); }
int main() {
  // Diagonal M(q)=diag(2+exp(q_i)), an energy-consistent variable-inertia plant.
  // c_i=0.5*exp(q_i)*dq_i^2. Independent finite differences check the sign/bias.
  hardware_cbf::Seven q{},v{},c{},u{},lo{},hi{},solution{};
  double energy=0;
  for (size_t i=0; i<7; ++i) {
    q[i]=.05*i; v[i]=.1+.02*i; u[i]=2+i;
    c[i]=.5*std::exp(q[i])*v[i]*v[i];
    energy+=.5*(2+std::exp(q[i]))*v[i]*v[i];
    lo[i]=-20; hi[i]=20;
  }
  const auto terms=totalEnergyBarrier(v,c);
  auto hdot = [&](const hardware_cbf::Seven& control) {
    double result=terms.b;
    for (size_t i=0; i<7; ++i) result+=terms.a[i]*control[i];
    return result;
  };
  auto finite_difference = [&](const hardware_cbf::Seven& control) {
    const double dt=1e-6;
    double ep=0,em=0;
    for (size_t i=0; i<7; ++i) {
      const double acc=control[i]/(2+std::exp(q[i]));
      ep+=.5*(2+std::exp(q[i]+dt*v[i]))*std::pow(v[i]+dt*acc,2);
      em+=.5*(2+std::exp(q[i]-dt*v[i]))*std::pow(v[i]-dt*acc,2);
    }
    return -(ep-em)/(2*dt);
  };
  require(std::abs(hdot(u)-finite_difference(u))<1e-8,"total barrier derivative mismatch");
  const double h=.01,alpha=1;
  require(hdot(u)+alpha*h<0,"fixture must require filtering");
  require(hardware_cbf::project(u,lo,hi,terms.a,-terms.b-alpha*h,solution),"QP must be feasible");
  require(finite_difference(solution)+alpha*h>-1e-8,"filtered total-energy derivative violates bound");
  hardware_cbf::Seven zero{};
  const auto at_rest=totalEnergyBarrier(zero,c);
  require(at_rest.b==0,"rest drift is zero");
  for (double a:at_rest.a) require(a==0,"rest coefficient is zero");
  std::cout<<"Total-energy derivative and constrained control: PASS\n";
}
