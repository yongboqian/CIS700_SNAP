#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "snap_low_level_detectors/detector_manager.h"

namespace snap_low_level_detectors
{
class DetectorManagerNodelet : public nodelet::Nodelet
{
    std::unique_ptr<DetectorManager> dm;
public:
    DetectorManagerNodelet() {}
    virtual ~DetectorManagerNodelet() {}
    virtual void onInit()
    {
        dm.reset(new DetectorManager(getNodeHandle(), getPrivateNodeHandle()));
    }
};

} // namespace snap_low_level_detectors

PLUGINLIB_EXPORT_CLASS(snap_low_level_detectors::DetectorManagerNodelet, nodelet::Nodelet)

