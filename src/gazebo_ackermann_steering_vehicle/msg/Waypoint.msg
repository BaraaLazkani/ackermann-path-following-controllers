# Waypoint message for path following
# Contains position, orientation, curvature, and reference velocity

float64 x           # x position in world frame [m]
float64 y           # y position in world frame [m]
float64 theta       # heading angle [rad]
float64 kappa       # path curvature [1/m]
float64 s           # arc length along path [m]
float64 v_ref       # reference velocity at this waypoint [m/s]
