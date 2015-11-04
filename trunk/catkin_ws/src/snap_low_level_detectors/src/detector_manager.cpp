//#include <pluginlib/class_loader.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <snap_vision_msgs/DetectionsStamped.h>
#include "snap_low_level_detectors/detector_manager.h"
#include "snap_low_level_detectors/STUB_detector.h"
#include "snap_low_level_detectors/ocv_cascade_detector.h"

using namespace snap_vision_msgs;
using namespace snap_low_level_detectors;

/** fix C++11 oversight, the easy way.
 * see http://stackoverflow.com/questions/7038357 for more details */
namespace std
{
template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args)
{
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
} // namespace std

DetectorManager::DetectorManager()
    : nh_("")
    , pnh_("~")
    , it_(nh_) /* should this be pnh_ ? */
    , sub_(/* no need to subscribe yet */)
    , pub_(pnh_.advertise<snap_vision_msgs::DetectionsStamped>(
                "detections", 10))
    , pDetector_(/* no need to initialize a detector yet */)
    ,   load_detector_(pnh_.advertiseService(  "load_detector",
                &DetectorManager::  loadDetectorCb, this))
    , unload_detector_(pnh_.advertiseService("unload_detector",
                &DetectorManager::unloadDetectorCb, this))
    ,   load_models_(pnh_.advertiseService(  "load_models",
                &DetectorManager::  loadModelsCb, this))
    , unload_models_(pnh_.advertiseService("unload_models",
                &DetectorManager::unloadModelsCb, this))
    //, load_params_(pnh_.advertiseService("load_params",
    //            &DetectorManager::loadParamsCb, this))
    //, save_params_(pnh_.advertiseService("save_params",
    //            &DetectorManager::saveParamsCb, this))
    , start_stream_(pnh_.advertiseService("start_stream",
                &DetectorManager::startStreamCb, this))
    ,  stop_stream_(pnh_.advertiseService( "stop_stream",
                &DetectorManager:: stopStreamCb, this))
{}

#define CHECK_LOADED \
    if(!pDetector_) { \
        res.error.err_code = Error::E_MODEL_NOT_LOADED; \
        res.error.err_str = "Detector not loaded"; \
        return true; \
    }

#define CHECK_UNLOADED \
    if(pDetector_) { \
        res.error.err_code = Error::E_MODEL_NOT_UNLOADED; \
        res.error.err_str = "Detector alroady loaded"; \
        return true; \
    }

#define CHECK_NOT_STREAMING \
    if(pDetector_) { \
        res.error.err_code = Error::E_MODEL_NOT_UNLOADED; \
        res.error.err_str = "Detector alroady loaded"; \
        return true; \
    }

bool DetectorManager::loadDetectorCb(Detector::Request &req,
                                     Detector::Response &res)
{
    CHECK_UNLOADED;

    try {
        if("STUBDetector" == req.type) {
            pDetector_ = std::make_unique<STUBDetector>();
        } else if("OCVCascadeDetector" == req.type) {
            pDetector_ = std::make_unique<OCVCascadeDetector>();
        } else {
            res.error.err_code = Error::E_DETECTOR_NOT_FOUND;
            res.error.err_str = "Detector type '"
                + req.type + "' not known.";
            return true;
        }

        pDetector_->init();
        res.error.err_code = Error::E_OK;
        res.error.err_str = "STUBDetector loaded";
        return true;
    } catch (const std::exception &e) {
        res.error.err_code = Error::E_UNKNOWN_ERROR;
        res.error.err_str = e.what();
        return true;
    }
}


bool DetectorManager::unloadDetectorCb(Detector::Request &req,
                                       Detector::Response &res)
{
    CHECK_LOADED;

    res.error.err_code = Error::E_OK;
    res.error.err_str = pDetector_->getType() + " unloaded";
    pDetector_->finalize();
    pDetector_.release();
    return true;
}

bool DetectorManager::loadModelsCb(Models::Request &req,
                                   Models::Response &res)
{
    CHECK_LOADED;

    res.error = pDetector_->loadModels(req.model_names);
    return true;
}

bool DetectorManager::unloadModelsCb(Models::Request &req,
                                     Models::Response &res)
{
    CHECK_LOADED;

    res.error = pDetector_->unloadModels(req.model_names);
    return true;
}

/*
bool DetectorManager::loadParamsCb(Params::Request &req,
                                   Params::Response &res)
{
    CHECK_LOADED;

    res.error = pDetector_->loadParams(req.filename);
    return true;
}

bool DetectorManager::saveParamsCb(Params::Request &req,
                                   Params::Response &res)
{
    CHECK_LOADED;

    res.error = pDetector_->saveParams(req.filename);
    return true;
}
*/

bool DetectorManager::startStreamCb(Stream::Request &req,
                                    Stream::Response &res)
{
    //CHECK_LOADED;
    //CHECK_NOT_STREAMING;
    sub_ = it_.subscribe(req.topic_name, req.queue_size, &DetectorManager::imageCb, this);
    res.error.err_code = Error::E_OK;
    return true;
}

bool DetectorManager::stopStreamCb(Stream::Request &req,
                                   Stream::Response &res)
{
    sub_.shutdown();
    res.error.err_code = Error::E_OK;
    return true;
}

void DetectorManager::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    if(!pDetector_) {
        ROS_ERROR_STREAM_THROTTLE(5, "Got an image on topic '"
                << sub_.getTopic() << "' but no detector is loaded.");
        return;
    }

    if(pub_.getNumSubscribers() == 0) {
        ROS_WARN_STREAM_THROTTLE(5, "No subscribers on '"
                << pub_.getTopic() << "' so not running detection.");
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    snap_vision_msgs::DetectionsStamped dets;
    dets.header = msg->header;

    pDetector_->detect(cv_ptr->image, dets.detections);
    pub_.publish(dets);
}

