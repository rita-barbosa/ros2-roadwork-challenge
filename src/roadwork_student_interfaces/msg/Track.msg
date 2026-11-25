# One radar track of a MOVING object (sense_node -> plan_node inside Scene).
# Frame: map (x along the road, y left). Units: m, m/s. Times: simulation time.
# Only detections whose ego-motion-compensated radial velocity exceeds the
# moving threshold create or update tracks; static returns (works, parked
# obstacles) are left to the LiDAR. Motion is assumed to be along the road (x),
# so the along-road speed is recovered from the radial velocity by
# vx = v_r,comp / cos(map bearing).

uint32 id
builtin_interfaces/Time stamp   # time of the state below (last radar update; NOT predicted)
float64 x                       # position at stamp
float64 y
float64 vx                      # along-road velocity (negative = towards the ego, oncoming)
uint32 hits                     # radar updates received
bool confirmed                  # hits >= confirm_hits; Plan ignores unconfirmed tracks
bool coasting                   # predicted outside the radar FOV/range: kept by constant-velocity prediction
