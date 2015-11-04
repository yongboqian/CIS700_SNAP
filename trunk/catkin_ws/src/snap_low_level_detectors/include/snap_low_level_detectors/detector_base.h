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

#include <dynamic_reconfigure/server.h>

#include <snap_vision_msgs/Detection.h>
#include <snap_vision_msgs/Error.h>

namespace snap_low_level_detectors
{

template <typename CONFIG>
class Reconfigurable
{
        dynamic_reconfigure::Server<CONFIG> dynparamServer_;
        typename dynamic_reconfigure::Server<CONFIG>::CallbackType dynparamCb_;

    protected:
        CONFIG config_;

    public:
        Reconfigurable()
        {
            dynparamCb_ = boost::bind(&Reconfigurable<CONFIG>::dynparamCb, this, _1, _2);
            dynparamServer_.setCallback(dynparamCb_);
        }

        void dynparamCb(CONFIG &config, uint32_t level) { config_ = config; }
}; // class Reconfigurable<CONFIG>
        
class DetectorBase
{
    public:
        /** Default constructor */
        DetectorBase() {}

        /** Destructor */
        virtual ~DetectorBase() {}

        /** Initialize and finalize */
        virtual void init() = 0;
        virtual void finalize() = 0;

        /** Parameters */
        //virtual snap_vision_msgs::Error loadParams(const std::string &fname);
        //virtual snap_vision_msgs::Error saveParams(const std::string &fname);

        /** Load/unload models */
        virtual snap_vision_msgs::Error   loadModels(const std::vector<std::string> &model_names) = 0;
        virtual snap_vision_msgs::Error unloadModels(const std::vector<std::string> &model_names) = 0;

        /** Detect */
        virtual void detect(const cv::InputArray &image,
                std::vector<snap_vision_msgs::Detection> &detections) = 0;

        /** Introspection */
        virtual std::string getType() const = 0;
}; // class DetectorBase

} // namespace snap_low_level_detectors
