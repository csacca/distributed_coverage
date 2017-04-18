# Node Entry/Exit message
# header: standard ROS header
# state: enum STATE_ENTRY, STATE_EXIT

Header header
string state
uint32 node_id

string STATE_ENTRY='entry'
string STATE_EXIT='exit'
