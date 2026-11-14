# Internal plan: plan_node -> act_node on /solution/plan.
# QoS: reliable, volatile, keep-last 1.
# header.stamp: plan creation time (simulation time). Act must reject the plan when
# now > valid_until, when the stamp is in the future, or when any value is non-finite,
# and then command speed 0 while holding steering.
# Units: m/s, rad (+ = left / counter-clockwise).

uint8 STATE_INIT=0            # waiting for odometry / beacon geometry
uint8 STATE_WAIT_GO=1         # no confirmed GO: own lane, stopping before the entry line
uint8 STATE_APPROACH=2        # GO confirmed, own lane before the entry line
uint8 STATE_BYPASS=3          # inside the work zone on the left bypass
uint8 STATE_RETURN=4          # returning to own lane before exit
uint8 STATE_FINAL=5           # own lane after exit, approaching the goal
uint8 STATE_GOAL=6            # stopped at goal (latched)
uint8 STATE_SAFE_STOP=7       # missing/stale odometry or scene: braking

std_msgs/Header header
builtin_interfaces/Time scene_stamp    # scene used
builtin_interfaces/Time odom_stamp     # ego pose used for steering
builtin_interfaces/Time valid_until    # expiry of this plan
float64 target_speed                   # >= 0
float64 target_steering                # front-wheel angle
float64 speed_limit                    # cap Act must enforce in addition to its own
bool hold                              # true: Plan requests speed 0
uint8 state
string state_name
string reason
