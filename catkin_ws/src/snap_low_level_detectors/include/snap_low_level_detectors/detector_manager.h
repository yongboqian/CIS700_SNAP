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
#include <cv_bridge/cv_bridge.h>
#include <snap_vision_msgs/Detector.h>
#include <snap_vision_msgs/Models.h>
#include <snap_vision_msgs/Stream.h>
#include <snap_vision_msgs/Params.h>

namespace snap_low_level_detectors
{

typedef std::vector<snap_vision_msgs::Detection> vDetection;

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

    /** Outputting the image with detections */
    image_transport::Publisher pub_viz_;
    /** Persist the colors used for each type of detection */
    std::map<std::string, cv::Scalar> dets_colors_;
    /** Temporary images/messages */
    cv_bridge::CvImage image_;
    sensor_msgs::Image image_msg_;
    cv::RNG rng_;

    /** Drawing detections */
    void visualizeAndPublish(const std_msgs::Header &header, const cv::Mat &image, const vDetection &dets);
    std::set<std::string> drawDetections(const cv::Mat &imgIn, const vDetection &dets, cv::Mat &imgOut);
    void overlayDetectionsLegend(cv::Mat &img, const std::set<std::string> &classes);

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
