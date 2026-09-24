std_msgs/Header header
# Raw CBF input/result without Coriolis; u_safe is not applied during abort.
float64[7] u_nom
float64[7] u_safe
float64[7] coriolis
float64[7] u_des
float64[7] u_cbf
float64[7] u_measured
float64[7] u_saturated
float64[7] u_ext
float64 h
uint8 solver_status
float64 kinetic_energy
float64 Kmax
float64 directional_kinetic_energy