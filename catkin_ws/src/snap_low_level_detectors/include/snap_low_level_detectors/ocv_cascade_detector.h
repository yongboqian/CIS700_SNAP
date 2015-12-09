#pragma once
/**
 * ocv_cascade_detector.h
 *
 * A cascade detector using OpenCV implementation.
 *
 * Author:        Nicu Stiurca
 * Email:         nstiurca@seas.upenn.edu
 * Creation date: 2015-11-03
 */

#include <map>
#include <opencv2/objdetect.hpp>
#include <snap_low_level_detectors/OCVCascadeDetectorConfig.h>
#include "detector_base.h"

namespace snap_low_level_detectors
{

class OCVCascadeDetector : public DetectorBase, private Reconfigurable<OCVCascadeDetectorConfig>
{
    /** The OpenCV cascade classifier */
    std::map<std::string, cv::CascadeClassifier> classifiers_;

    /** Temporaries */
    cv::UMat imageGray_;
    std::vector<cv::Rect> objects_;
    std::vector<int> numDetections_;

public:
    /** Constructor and destructor */
    OCVCascadeDetector() {}
    virtual ~OCVCascadeDetector() {}
    
    /** Initialize and finalize */
    virtual void init();
    virtual void finalize();

    /** Parameters */
    //virtual snap_vision_msgs::Error loadParams(const std::string &fname);
    //virtual snap_vision_msgs::Error saveParams(const std::string &fname);

    /** Load/unload models */
    virtual snap_vision_msgs::Error   loadModels(const std::vector<std::string> &model_names);
    virtual snap_vision_msgs::Error unloadModels(const std::vector<std::string> &model_names);

    /** Detect */
    virtual void detect(const cv::InputArray &image, std::vector<snap_vision_msgs::Detection> &detections);

    /** Introspection */
    virtual std::string getType() const { return "OCVCascadeDetector"; }

}; // class OCVCascadeDetector

} // namespace snap_low_level_detectors
