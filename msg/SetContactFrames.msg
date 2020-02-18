string ADD=ADD
string REMOVE=REMOVE
string SET=SET

string action


string[] frames_in_contact

# rotations and friction_coefficient are used only for friction cones

geometry_msgs/Quaternion[] rotations
float64 friction_coefficient
bool optimize_torque # optimize contact moments
