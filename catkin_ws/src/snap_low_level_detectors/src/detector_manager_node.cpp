#include <ros/ros.h>
#include "snap_low_level_detectors/detector_manager.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detector_manager_node");

    snap_low_level_detectors::DetectorManager dm;

    ros::spin();

    return 0;
}

