#include "polishingrobot_onlineplanner/aubo_10_polishing_opreating.h"
using aubo10_polishing_control::Aubo10Polishing;

void Aubo10Polishing::Aubo10Polishing()
{
    n_private = ros::NodeHandle("aubo_10_polishing_opreating");
}
void Aubo10Polishing::Pub_Sub_Setup() {
  feature_sub = n_private.subscribe ("smarteye_shortest_path_point_output", 1, cloud_cb);
  
}
