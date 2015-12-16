#include <opencv2/imgproc.hpp>
#include <snap_vision_msgs/Models.h>
#include "snap_low_level_detectors/ocv_cascade_detector.h"

using namespace std;
using namespace cv;
using namespace snap_vision_msgs;
using namespace snap_low_level_detectors;

typedef snap_vision_msgs::Error SnapError;

const static map<string, string> s_cascadeFilename{
    {"frontalface", "/opt/opencv3/share/OpenCV/lbpcascades/lbpcascade_frontalface.xml"},
    {"profileface", "/opt/opencv3/share/OpenCV/lbpcascades/lbpcascade_profileface.xml"},
    {"silverware", "/opt/opencv3/share/OpenCV/lbpcascades/lbpcascade_silverware.xml"},
    {"marker", "/opt/opencv3/share/OpenCV/custom_cascades/snap/marker.xml"},
    {"duck", "/opt/opencv3/share/OpenCV/custom_cascades/snap/duck.xml"},
    {"duck_32x32_haar", "/opt/opencv3/share/OpenCV/custom_cascades/snap/duck_32x32_haar.xml"},
    {"tape_32x32_haar", "/opt/opencv3/share/OpenCV/custom_cascades/snap/tape_32x32_haar.xml"},
    {"bottle_24x32_haar", "/opt/opencv3/share/OpenCV/custom_cascades/snap/bottle_24x32_haar.xml"},
    {"duck_16x16_haar", "/opt/opencv3/share/OpenCV/custom_cascades/snap/duck_16x16_haar.xml"},
    {"tape_16x16_haar", "/opt/opencv3/share/OpenCV/custom_cascades/snap/tape_16x16_haar.xml"},
    {"bottle_12x16_haar", "/opt/opencv3/share/OpenCV/custom_cascades/snap/bottle_12x16_haar.xml"},
    {"bot_16x16_HAAR",    "/opt/opencv3/share/OpenCV/custom_cascades/snap/classifier_tight_bot_16x16_HAAR.xml"},
    {"bot_16x16_LBP",     "/opt/opencv3/share/OpenCV/custom_cascades/snap/classifier_tight_bot_16x16_LBP.xml"},
    {"bottle_16x16_HAAR", "/opt/opencv3/share/OpenCV/custom_cascades/snap/classifier_tight_bottle_16x16_HAAR.xml"},
    {"bottle_16x16_LBP",  "/opt/opencv3/share/OpenCV/custom_cascades/snap/classifier_tight_bottle_16x16_LBP.xml"},
    {"duck_16x16_HAAR",   "/opt/opencv3/share/OpenCV/custom_cascades/snap/classifier_tight_duck_16x16_HAAR.xml"},
    {"duck_16x16_LBP",    "/opt/opencv3/share/OpenCV/custom_cascades/snap/classifier_tight_duck_16x16_LBP.xml"},
    {"duck_24x24_HAAR",   "/opt/opencv3/share/OpenCV/custom_cascades/snap/classifier_tight_duck_24x24_HAAR.xml"},
    {"duck_24x24_LBP",    "/opt/opencv3/share/OpenCV/custom_cascades/snap/classifier_tight_duck_24x24_LBP.xml"},
    {"tape_16x16_HAAR",   "/opt/opencv3/share/OpenCV/custom_cascades/snap/classifier_tight_tape_16x16_HAAR.xml"},
    {"tape_16x16_LBP",    "/opt/opencv3/share/OpenCV/custom_cascades/snap/classifier_tight_tape_16x16_LBP.xml"},
};

void OCVCascadeDetector::init()
{
    //scaleFactor_ = 1.1;
    //minNeighbors_ = 3;
    //flags_ = 0;
    //minSize_ = maxSize_ = Size();
}

void OCVCascadeDetector::finalize()
{
    classifiers_.clear();
    imageGray_.release();
    objects_.clear();
    numDetections_.clear();
}

static const std::vector<std::string>& resolveModelNames(const std::vector<std::string> &model_names)
{
    ROS_INFO("Resolving model names...");
    static std::vector<std::string> all_model_names;
    if(all_model_names.empty()) {
        for(auto &kv : s_cascadeFilename) {
            all_model_names.push_back(kv.first);
            ROS_INFO_STREAM("Known model: " << all_model_names.back());
        }
    }

    if(model_names.size() == 1 && model_names[0] == snap_vision_msgs::Models::Request::ALL) {
        ROS_INFO("Resolving model names... DONE1");
        return all_model_names;
    } else {
        ROS_INFO("Resolving model names... DONE2");
        return model_names;
    }
}

SnapError OCVCascadeDetector::loadModels(const std::vector<std::string> &model_names)
{
    ROS_INFO("Loading models...");
    size_t num_loaded = 0;
    size_t num_failed = 0;
    SnapError ret;
    ret.err_code = SnapError::E_OK;

    for(const string &model_name : resolveModelNames(model_names)) {
        CascadeClassifier &cc = classifiers_[model_name];   // std::map::operator[] default-constructs as needed
        if(cc.empty()) {
            try {
                const string &filename = s_cascadeFilename.at(model_name);
                bool loaded = cc.load(filename);
                if(loaded) {
                    ++num_loaded;
                } else {
                    ++num_failed;
                    ret.err_code |= SnapError::E_FILE_NOT_FOUND;
                }
            } catch (const std::out_of_range &e) {
                ++num_failed;
                ret.err_code |= SnapError::E_MODEL_NOT_FOUND;
            }
        } else { // if(cc.empty())
            // model already loaded, so nothing to do
        }
    }
    char buf[80];
    snprintf(buf, sizeof(buf), "%zu out of %zu models loaded (%zu failed).",
            num_loaded, model_names.size(), num_failed);
    ret.err_str = buf;
    ROS_INFO("Loading models... DONE");
    return ret;
}

SnapError OCVCascadeDetector::unloadModels(const std::vector<std::string> &model_names)
{
    ROS_INFO("Loading models...");
    size_t num_unloaded = 0;
    for(const string &model_name : resolveModelNames(model_names)) {
        num_unloaded += classifiers_.erase(model_name);
    }
    char buf[80];
    snprintf(buf, sizeof(buf), "%zu out of %zu models unloaded.", num_unloaded, model_names.size());

    SnapError ret;
    ret.err_code = SnapError::E_OK;
    ret.err_str = buf;
    ROS_INFO("Unloading models... DONE");
    return ret;
}

void OCVCascadeDetector::detect(const InputArray &image, vector<Detection> &detections)
{
    ROS_INFO("Detecting...");
    CV_Assert(image.type() == CV_8UC3);
    cvtColor(image, imageGray_, CV_BGR2GRAY);

    detections.clear();

    for(auto &kv : classifiers_) {
        if(kv.second.empty()) continue;

        Detection det;
        det.label = kv.first;

        kv.second.detectMultiScale(imageGray_, objects_, numDetections_,
                config_.scale_factor,
                config_.min_neighbors,
                config_.flags,
                Size(config_.min_width, config_.min_height),
                Size(config_.max_width, config_.max_height));

        for(size_t i=0; i<objects_.size(); ++i) {
            det.bbox.x = objects_[i].x;
            det.bbox.y = objects_[i].y;
            det.bbox.width = objects_[i].width;
            det.bbox.height = objects_[i].height;
            det.confidence = numDetections_[i];
            detections.push_back(det);
        }
    }
    ROS_INFO("Detecting... DONE");
}
