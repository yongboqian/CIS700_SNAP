#pragma once
/**
 * STUB_detector.h
 *
 * A STUB of a detector, just to have something instantiatable.
 *
 * Author:        Nicu Stiurca
 * Email:         nstiurca@seas.upenn.edu
 * Creation date: 2015-11-03
 */

#include "detector_base.h"

namespace snap_low_level_detectors
{

class STUBDetector : public DetectorBase
{
    static snap_vision_msgs::Error E_STUB()
    {
        snap_vision_msgs::Error err;
        err.err_code = snap_vision_msgs::Error::E_STUB;
        err.err_str = "This is a STUB originating in class STUBDetector.";
        return err;
    }

    public:
        /** Initialize and finalize */
        virtual void init() {}
        virtual void finalize() {}

        /** Parameters */
        virtual snap_vision_msgs::Error loadParams(const std::string &fname) { return E_STUB(); }
        virtual snap_vision_msgs::Error saveParams(const std::string &fname) { return E_STUB(); }

        /** Load/unload models */
        virtual snap_vision_msgs::Error   load_models(const std::vector<std::string> &model_names)
            { return E_STUB(); }
        virtual snap_vision_msgs::Error unload_models(const std::vector<std::string> &model_names)
            { return E_STUB(); }

        /** Detect */
        virtual void detect(const cv::InputArray &image, std::vector<snap_vision_msgs::Detection> &detections) { detections.clear(); }

        /** Introspection */
        virtual std::string getType() const { return "STUBDetector"; }

}; // class STUBDetector

} // namespace snap_low_level_detectors
