#include <string>

#include <boost/filesystem.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

class DatasetCollector
{
    // an enum
    enum EMouseMode
    {
        MARK_NONE,
        MARK_BBOX,
        MARK_FG,
        MARK_BG,
    };

    // ROS stuff
    ros::NodeHandle nh_, pnh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    // callbacks
    void imageCb(const sensor_msgs::ImageConstPtr &msg);
    std::string resolved_topic_name_;

    // parameters
    boost::filesystem::path save_dir_;
    std::string prefix_;
    int count_;
    std::string extension_;

    // temporaries
    bool new_frame_available_, new_grabbed_frame_available_;
    bool new_cropped_frame_available_, new_segmented_frame_available_;
    cv_bridge::CvImageConstPtr cv_current_frame_ptr_;
    cv_bridge::CvImageConstPtr cv_grabbed_frame_ptr_;
    std::string grabbed_frame_fname_;
    cv::Mat grabbed_frame_viz_;
    cv::Mat cropped_frame_;
    cv::Mat segmented_frame_;
    cv::Rect bbox_, tight_bbox_;
    cv::Point top_left_, bottom_right_;
    cv::Mat mask_, mask;
    cv::Mat background_model_;
    cv::Mat foreground_model_;
    EMouseMode mouse_mode_;
    cv::Point mouse_pos_;

    // helper methods
    void updateGUI();
    void doUserIO();
    void saveCurrentGrabbedFrame();
    void cropCurrentFrame();
    void grabbedFrameMouseCb(int event, int x, int y, int flags);
    void grabCurrentFrame();
    void grabCut(int mode);

    void markBboxMouseCb(int event, int x, int y, int flags);
    void markMaskMouseCb(int event, int x, int y, int flags);

    bool isMaskValid() const;
    bool isBboxValid() const;

    // state changes
    void markBbox();
    void markForeground();
    void markBackground();

    // friends can see your privates
    friend void grabCurrentFrameButtonCb(int state, void *self);
    friend void saveCurrentGrabbedFrameButtonCb(int state, void *self);
    friend void grabbedFrameMouseCb(int event, int x, int y, int flags, void *self);

public:
    DatasetCollector();
    ~DatasetCollector();
    void spin();

}; // class DatasetCollector()

