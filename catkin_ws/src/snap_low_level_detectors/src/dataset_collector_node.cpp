#include "dataset_collector.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dataset_collector", true);

    DatasetCollector dc;
    dc.spin();
    ros::shutdown();

    return 0;
}

