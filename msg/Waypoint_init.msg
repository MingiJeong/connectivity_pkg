std_msgs/Header header

int32 auction_round # current auction round
string leader_name  # leader robot name
string text_info  # leader string message
geometry_msgs/Polygon waypoints  # intended waypoints
bool[] allocation_status # allocation status of each point