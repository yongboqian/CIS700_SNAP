#include <ros/ros.h>
#include "snap_low_level_detectors/detector_manager.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detector_manager_node");

    ros::NodeHandle nh(""), pnh("~");
    snap_low_level_detectors::DetectorManager dm(nh, pnh);

    ros::spin();

    return 0;
}

