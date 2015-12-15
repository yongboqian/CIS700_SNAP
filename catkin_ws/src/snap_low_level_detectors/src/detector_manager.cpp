//#include <pluginlib/class_loader.h>
#include <opencv2/imgproc.hpp>
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

static inline ros::Duration operator*(double t, const ros::Duration &d)
    { return d * t; }

#define DEBUG(x) ROS_DEBUG_STREAM(#x << ": " << x)
#define INFO(x) ROS_INFO_STREAM(#x << ": " << x)

DetectorManager::DetectorManager(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : nh_(nh)
    , pnh_(pnh)
    , it_(pnh_) /* should this be nh_ ? */
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
    , pub_viz_(it_.advertise("detections_vizualization", 10))
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
        res.error.err_str = req.type + " loaded";
        return true;
    } catch (const std::exception &e) {
        res.error.err_code = Error::E_UNKNOWN_ERROR;
        res.error.err_str = e.what();
        return true;
    }

    rng_ = cv::RNG(0);
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
    const ros::Time receive_time = ros::Time::now();
    if(!pDetector_) {
        ROS_ERROR_STREAM_THROTTLE(5, "Got an image on topic '"
                << sub_.getTopic() << "' but no detector is loaded.");
        return;
    }

    if(pub_.getNumSubscribers() == 0 && pub_viz_.getNumSubscribers() == 0) {
        ROS_WARN_STREAM_THROTTLE(5, "No subscribers on '"
                << pub_.getTopic() << "' nor '"
                << pub_viz_.getTopic() << "' so not running detection.");
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        // FIXME: use toCvShare, but figure out why that causes segfaults
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    snap_vision_msgs::DetectionsStamped dets;
    dets.header = msg->header;

    const ros::Time decode_time = ros::Time::now();
    pDetector_->detect(cv_ptr->image, dets.detections);
    const ros::Time detect_time = ros::Time::now();
    pub_.publish(dets);
    const ros::Time publish_time = ros::Time::now();

    visualizeAndPublish(msg->header, cv_ptr->image, dets.detections);
    const ros::Time visualize_time = ros::Time::now();

    static ros::Duration receive_duration   = receive_time - msg->header.stamp;
    static ros::Duration decode_duration    = decode_time - receive_time;
    static ros::Duration detect_duration    = detect_time - decode_time;
    static ros::Duration publish_duration   = publish_time - detect_time;
    static ros::Duration publish_latency    = publish_time - msg->header.stamp;
    static ros::Duration visualize_duration = visualize_time - publish_time;

    const double alpha = 0.5;
    const double beta = 1.0 - alpha;
    receive_duration   = alpha*receive_duration   + beta*(receive_time - msg->header.stamp);
    decode_duration    = alpha*decode_duration    + beta*(decode_time - receive_time)      ;
    detect_duration    = alpha*detect_duration    + beta*(detect_time - decode_time)       ;
    publish_duration   = alpha*publish_duration   + beta*(publish_time - detect_time)      ;
    publish_latency    = alpha*publish_latency    + beta*(publish_time - msg->header.stamp);
    visualize_duration = alpha*visualize_duration + beta*(visualize_time - publish_time)   ;


    DEBUG(receive_duration);
    DEBUG(decode_duration);
    INFO(detect_duration);
    DEBUG(publish_duration);
    DEBUG(visualize_duration);
    INFO(publish_latency);
}

void DetectorManager::visualizeAndPublish(const std_msgs::Header &header,
        const cv::Mat &image, const vDetection &dets)
{
    if(pub_viz_.getNumSubscribers() == 0) {
        ROS_INFO_STREAM_THROTTLE(5, "No subscribers on '"
                << pub_viz_.getTopic() << "' so not visualizing.");
        return;
    }

    image_.header = header;
    image_.encoding = "bgr8";
    const std::set<std::string> classes = drawDetections(image, dets, image_.image);
    overlayDetectionsLegend(image_.image, classes);
    image_.toImageMsg(image_msg_);
    pub_viz_.publish(image_msg_);
}

static cv::Scalar randomColor( cv::RNG& rng )
{
    int icolor = (unsigned) rng;
    return cv::Scalar( icolor&255, (icolor>>8)&255, (icolor>>16)&255 );
}

std::set<std::string> DetectorManager::drawDetections(const cv::Mat &imgIn, const vDetection &dets,
        cv::Mat &imgOut)
{
    const int shift = 2;
    const float mul = 1<<shift;
    const int thickness = 2;
    const int lineType = cv::LINE_8;

    std::set<std::string> classes;

    imgIn.copyTo(imgOut);

    for(auto &det : dets) {
        classes.insert(det.label);

        cv::Scalar &color = dets_colors_[det.label];
        if(cv::Scalar() == color) {
            // first time seeing this label, so generate a new color
            color = randomColor(rng_);
        }

        // TODO: do we need to check bounds?
        cv::rectangle(imgOut, cv::Rect(cvRound(det.bbox.x*mul), cvRound(det.bbox.y*mul),
                cvRound(det.bbox.width*mul), cvRound(det.bbox.height*mul)),
                color, (int)(std::ceil(std::sqrt(thickness*det.confidence))), lineType, shift);
    }

    return classes;
}

void DetectorManager::overlayDetectionsLegend(cv::Mat &img,
        const std::set<std::string> &classes)
{
    const int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    const double fontScale = 1.0;
    const int thickness = 2;
    const int lineType = cv::LINE_8;
    const bool bottomLeftOrigin = false;

    cv::Point org(10,30);
    for(auto &cls : classes) {
        const cv::Scalar &color = dets_colors_.at(cls);
        
        cv::putText(img, cls, org, fontFace, fontScale, color,
                thickness, lineType, bottomLeftOrigin);

        org.y += 25;
    }
}
