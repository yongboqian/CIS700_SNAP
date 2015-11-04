#pragma once
/**
 * detector_manager.h
 *
 * Class for flexible, low level object detection with a ROS interface.
 *
 * Author:        Nicu Stiurca
 * Email:         nstiurca@seas.upenn.edu
 * Creation date: 2015-11-02
 */
#include <memory>

#include "detector_base.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <snap_vision_msgs/Detector.h>
#include <snap_vision_msgs/Models.h>
#include <snap_vision_msgs/Stream.h>
#include <snap_vision_msgs/Params.h>

namespace snap_low_level_detectors
{

class DetectorManager
{
    /** Nodes, including a private one. */
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    /** Image subscription. TODO: Maybe multiple subscribers later? */
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_;

    /** Detection publisher. TODO: Maybe multiple publishers later? */
    ros::Publisher pub_;

    /** Some detector */
    std::unique_ptr<DetectorBase> pDetector_;

    /** Services to allow dynamically managing detectors */
    ros::ServiceServer   load_detector_,
                       unload_detector_,
                         load_models_,
                       unload_models_,
                       load_params_,
                       save_params_,
                       start_stream_,
                        stop_stream_;


public:
    /** Constructor */
    DetectorManager();
    /** Destructor */
    ~DetectorManager() {}

    /** Service callbacks */
    bool   loadDetectorCb(snap_vision_msgs::Detector::Request &req,
                          snap_vision_msgs::Detector::Response &res);
    bool unloadDetectorCb(snap_vision_msgs::Detector::Request &req,
                          snap_vision_msgs::Detector::Response &res);

    bool   loadModelsCb(snap_vision_msgs::Models::Request &req,
                        snap_vision_msgs::Models::Response &res);
    bool unloadModelsCb(snap_vision_msgs::Models::Request &req,
                        snap_vision_msgs::Models::Response &res);

    //bool loadParamsCb(snap_vision_msgs::Params::Request &req,
    //                  snap_vision_msgs::Params::Response &res);
    //bool saveParamsCb(snap_vision_msgs::Params::Request &req,
    //                  snap_vision_msgs::Params::Response &res);
 
    bool startStreamCb(snap_vision_msgs::Stream::Request &req,
                       snap_vision_msgs::Stream::Response &res);
    bool  stopStreamCb(snap_vision_msgs::Stream::Request &req,
                       snap_vision_msgs::Stream::Response &res);
   
    /** Streaming image callback */
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

}; // class DetectorManager

} // namespace snap_low_level_detectors
