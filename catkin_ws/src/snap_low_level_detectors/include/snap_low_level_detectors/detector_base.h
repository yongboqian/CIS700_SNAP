#pragma once
/**
 * detector_base.h
 *
 * Base class for low level object detection.
 *
 * Author:        Nicu Stiurca
 * Email:         nstiurca@seas.upenn.edu
 * Creation date: 2015-11-02
 */

#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

#include <snap_vision_msgs/Detection.h>
#include <snap_vision_msgs/Error.h>

namespace snap_low_level_detectors
{

class DetectorBase
{
    public:
        /** Default constructor */
        DetectorBase();

        /** Destructor */
        virtual ~DetectorBase() { finalize(); }

        /** Initialize and finalize */
        virtual void init();
        virtual void finalize();

        /** Parameters */
        virtual snap_vision_msgs::Error loadParams(const std::string &fname);
        virtual snap_vision_msgs::Error saveParams(const std::string &fname);

        /** Load/unload models */
        virtual snap_vision_msgs::Error   loadModels(const std::vector<std::string> &model_names);
        virtual snap_vision_msgs::Error unloadModels(const std::vector<std::string> &model_names);

        /** Detect */
        virtual void detect(const cv::InputArray &image,
                std::vector<snap_vision_msgs::Detection> &detections);

        /** Introspection */
        virtual std::string getType() const;
}; // class DetectorBase

} // namespace snap_low_level_detectors
