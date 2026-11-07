# Internal scene: sense_node -> plan_node on /solution/scene.
# QoS: reliable, volatile, keep-last 1 (only the newest scene is meaningful).
# Frames: map (x forward along road, y left). Units SI. All times are simulation time.
# header.stamp: time Sense assembled the scene. Plan rejects a scene older than its
# scene_timeout (default 0.25 s). Evidence blocks keep their ORIGINAL sensor stamps.

uint8 PANEL_UNKNOWN=0
uint8 PANEL_GO=1
uint8 PANEL_STOP=2

std_msgs/Header header

# ---- Ego state (/vehicle/odometry; exact in this simulator) ----
bool ego_valid                   # odometry received and age <= odom_timeout
builtin_interfaces/Time odom_stamp
float64 ego_x
float64 ego_y
float64 ego_yaw
float64 ego_speed

# ---- Panel camera (/camera/image_raw) ----
uint8 panel_raw                  # class of the newest accepted frame
uint8 panel_state                # STOP immediately; GO after N consecutive frames; stale/ambiguous is UNKNOWN
builtin_interfaces/Time camera_stamp
float32 panel_green_fraction     # fraction of inner-disc pixels classified green
uint32 panel_go_count            # consecutive GO frames (10 Hz)
uint32 camera_rejected           # cumulative malformed / repeated / future frames

# ---- Roadwork beacon (/infra/roadwork/beacon): cached work-zone geometry ----
bool geometry_known              # at least one valid beacon received
builtin_interfaces/Time beacon_stamp
float64 entry_x
float64 work_start
float64 work_end
float64 exit_x
float64 speed_limit
int8 pass_side
uint32 beacon_rejected
