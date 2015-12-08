#include "dataset_collector.h"
#include <fstream>

const int DRAW_WIDTH = 5;


void grabCurrentFrameButtonCb(int state, void *self)
{
    CV_Assert(!!self);
    CV_Assert(-1 == state);
    ((DatasetCollector*)self) -> grabCurrentFrame();
}

void saveCurrentGrabbedFrameButtonCb(int state, void *self)
{
    CV_Assert(!!self);
    CV_Assert(-1 == state);
    ((DatasetCollector*)self) -> saveCurrentGrabbedFrame();
}

void grabbedFrameMouseCb(int event, int x, int y, int flags, void *self)
{
    CV_Assert(!!self);
    ((DatasetCollector*)self) -> grabbedFrameMouseCb(event, x, y, flags);
}

DatasetCollector::DatasetCollector()
    : nh_("")
    , pnh_("~")
    , it_(nh_)
    , resolved_topic_name_(ros::names::resolve("image"))
    , new_frame_available_(false)
    , new_grabbed_frame_available_(false)
    , new_cropped_frame_available_(false)
    , new_segmented_frame_available_(false)
    , bbox_(-1, -1, -1, -1)
    , top_left_(-1, -1)
    , bottom_right_(-1, -1)
    , mouse_mode_(MARK_NONE)
{
    // get parameters
    std::string save_dir;
    pnh_.param<std::string>("save_dir", save_dir, "");
    pnh_.param<std::string>("prefix", prefix_, "image");
    pnh_.param<int>("start_count", count_, 0);
    pnh_.param<std::string>("extension", extension_, "png");

    // prepare save directory
    boost::filesystem::path abs_dir_path = boost::filesystem::absolute(save_dir);
    if(boost::filesystem::create_directories(abs_dir_path)) {
        ROS_INFO_STREAM("Created directory \"" << save_dir << "\" (aka "
            << abs_dir_path << ")");
    }
    save_dir_ = boost::filesystem::canonical(abs_dir_path);
    ROS_INFO_STREAM("Saving images to directory " << save_dir_ << ".");

    // set up OpenCV interface
    cv::namedWindow(resolved_topic_name_, CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_NORMAL);
    cv::namedWindow("grabbed frame", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_NORMAL);
    cv::namedWindow("cropped frame", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_NORMAL);
    cv::namedWindow("segmented frame", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_NORMAL);
    cv::createButton("Grab frame", grabCurrentFrameButtonCb, this, cv::QT_PUSH_BUTTON);
    cv::createButton("Save grabbed frame", saveCurrentGrabbedFrameButtonCb, this, cv::QT_PUSH_BUTTON);
    cv::setMouseCallback("grabbed frame", ::grabbedFrameMouseCb, this);
    ROS_INFO("Controls:\n"
             "\tq/Q/ESC\t\tQuit\n"
             "\tSPACE\t\tGrab current frame\n"
             "\ts/S\t\tSave grabbed frame\n"
             "\tc/C\t\tCrop grabbed frame\n"
             "\tf/F\t\tMark foreground\n"
             "\tb/B\t\tMack background\n"
             "\tg/G\t\tGrab cut segmentation");

    // subscribe
    image_sub_ = it_.subscribe(resolved_topic_name_, 3, &DatasetCollector::imageCb, this);
}

DatasetCollector::~DatasetCollector()
{
    image_sub_.shutdown();

    cv::destroyAllWindows();
}

void DatasetCollector::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
      cv_current_frame_ptr_ = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    new_frame_available_ = true;
}

void DatasetCollector::spin()
{
    ros::Rate rate(30);

    while(ros::ok()) {
        doUserIO();
        updateGUI();
        ros::spinOnce();
        rate.sleep();
    }
}

static const cv::Scalar bg(0,0,0);
static const cv::Scalar fg(0,255,0);
static const cv::Scalar prBg(0,0,255);
static const cv::Scalar prFg(0,255,255);

static bool drawMaybe(cv::Mat &m, const cv::Point &pt, const int radius = DRAW_WIDTH,
        const cv::Scalar color = cv::Scalar(255,0,0), const int thickness = -1,
        const int lineType = cv::LINE_8, const int shift=0)
{
    if(        0 <= pt.x && pt.x < m.cols
            && 0 <= pt.y && pt.y < m.rows) {
        cv::circle(m, pt, radius, color, thickness, lineType, shift);
        return true;
    } else {
        return false;
    }
}

static void overlayMask(cv::Mat &img, const cv::Mat &mask, cv::Rect &tight_bbox)
{
    cv::Mat_<cv::Vec3b> i = img; //.getMat(cv::ACCESS_RW);
    cv::Mat_<uint8_t> m = mask; //.getMat(cv::ACCESS_READ);

    int x1 = 0;
    int y1 = 0;
    int x0 = i.cols-1;
    int y0 = i.rows-1;

#pragma omp parallel for reduction(min:x0,y0) reduction(max:y1,y1)
    for(int y=0; y<i.rows; ++y) {
        for(int x=0; x<i.cols; ++x) {
            cv::Vec3b &bgr = i(y,x);
            cv::Scalar newColor(bgr[0], bgr[1], bgr[2]);
            switch(m(y,x)) {
                case cv::GC_BGD:
                    newColor = (newColor + bg) / 2;
                    break;
                case cv::GC_FGD:
                    newColor = (newColor + fg) / 2;
                    x0 = std::min(x0, x);
                    y0 = std::min(y0, y);
                    x1 = std::max(x1, x);
                    y1 = std::max(y1, y);
                    break;
                case cv::GC_PR_BGD:
                    newColor = (newColor + prBg) / 2;
                    break;
                case cv::GC_PR_FGD:
                    newColor = (newColor + prFg) / 2;
                    x0 = std::min(x0, x);
                    y0 = std::min(y0, y);
                    x1 = std::max(x1, x);
                    y1 = std::max(y1, y);
                    break;
            } // switch(m(y,x))
            bgr[0] = newColor[0];
            bgr[1] = newColor[1];
            bgr[2] = newColor[2];
        } // for(int x=0; x<i.cols; ++x)
    } // for(int y=0; y<i.rows; ++y)

    tight_bbox.x = x0;
    tight_bbox.y = y0;
    tight_bbox.width = x1 - x0 + 1;
    tight_bbox.height = y1 - y0 + 1;
} // static void overlayMask(...)

void DatasetCollector::updateGUI()
{
    if(new_frame_available_) {
        cv::imshow(image_sub_.getTopic(), cv_current_frame_ptr_->image);
        new_frame_available_ = false;
        //cv::updateWindow(image_sub_.getTopic());
    }

    if(new_grabbed_frame_available_ && !!cv_grabbed_frame_ptr_) {
        cv_grabbed_frame_ptr_->image.copyTo(grabbed_frame_viz_);

        if(isMaskValid()) {
            overlayMask(grabbed_frame_viz_, mask_, tight_bbox_);
            cv::rectangle(grabbed_frame_viz_, tight_bbox_, cv::Scalar(0, 255, 255), 1);
        }

        bool corners_valid = drawMaybe(grabbed_frame_viz_, top_left_)
                          && drawMaybe(grabbed_frame_viz_, bottom_right_);

        if(corners_valid) {
            cv::rectangle(grabbed_frame_viz_, top_left_, bottom_right_,
                    cv::Scalar(0, 255, 0), 2);
        }

        if(MARK_FG == mouse_mode_) {
            drawMaybe(grabbed_frame_viz_, mouse_pos_, DRAW_WIDTH, fg, 2);
        } else if(MARK_BG == mouse_mode_) {
            drawMaybe(grabbed_frame_viz_, mouse_pos_, DRAW_WIDTH, bg, 2);
        }

        cv::imshow("grabbed frame", grabbed_frame_viz_);
        new_grabbed_frame_available_ = false;
        //cv::updateWindow("grabbed frame");
    }

    if(new_cropped_frame_available_) {
        cv::imshow("cropped frame", cropped_frame_);
        new_cropped_frame_available_ = false;
        // cv::updateWindow("cropped frame");
    }

    if(new_segmented_frame_available_) {
        CV_Assert(!!cv_grabbed_frame_ptr_);
        CV_Assert(isBboxValid());
        CV_Assert(isMaskValid());
        cropped_frame_.copyTo(segmented_frame_);
        cv::compare(mask_(bbox_), cv::GC_BGD, mask, cv::CMP_EQ);
        segmented_frame_.setTo(cv::Scalar(0), mask);
        cv::compare(mask_(bbox_), cv::GC_PR_BGD, mask, cv::CMP_EQ);
        segmented_frame_.setTo(cv::Scalar(0), mask);
        cv::imshow("segmented frame", segmented_frame_);
        new_segmented_frame_available_ = false;
        // cv::updateWindow("segmented frame");
    }
}

void DatasetCollector::doUserIO()
{
    int k = cv::waitKey(3);
    ROS_DEBUG_STREAM("key: " << k);
    switch(k&0xFF) {
        case 27:    // ESC key
        case 'q':
        case 'Q':
            ROS_INFO("That's all, folks!");
            ros::shutdown();
            break;
        case ' ':
            grabCurrentFrame();
            break;
        case 's':
        case 'S':
            saveCurrentGrabbedFrame();
            break;
        case 'c':
        case 'C':
            markBbox();
            break;
        case 'g':
        case 'G':
            grabCut(cv::GC_EVAL);
            break;
        case 'f':
        case 'F':
            markForeground();
            break;
        case 'b':
        case 'B':
            markBackground();
            break;
    }
}

void DatasetCollector::grabCurrentFrame()
{
    // grab it
    CV_Assert(!!cv_current_frame_ptr_);
    cv_grabbed_frame_ptr_ = cv_current_frame_ptr_;

    // name it
    char buf[80];
    snprintf(buf, sizeof buf, "%s_%04d_%u.%09u.%s", prefix_.c_str(), count_,
            cv_grabbed_frame_ptr_->header.stamp.sec,
            cv_grabbed_frame_ptr_->header.stamp.nsec, extension_.c_str());
    grabbed_frame_fname_ = buf;
    cv::displayStatusBar("grabbed frame", "Grabbed \"" + grabbed_frame_fname_ + "\" (UNSAVED)");

    // update counters/flags
    ++count_;
    new_grabbed_frame_available_ = true;
    mask_.release();
    markBbox();
}

void DatasetCollector::markBbox()
{
    bbox_ = cv::Rect(-1, -1, -1, -1);
    top_left_ = cv::Point(-1, -1);
    bottom_right_ = cv::Point(-1, -1);
    mouse_mode_ = MARK_BBOX;
}

static const std::vector<int> compression_params{CV_IMWRITE_PNG_COMPRESSION, 9};

void DatasetCollector::saveCurrentGrabbedFrame()
{
    CV_Assert(!!cv_grabbed_frame_ptr_);
    
    /** Save the grabbed frame, without cropping or anything */
    boost::filesystem::path fname_full = save_dir_ / grabbed_frame_fname_;
    ROS_INFO_STREAM("Saving grabbed frame " << fname_full);

    cv::imwrite(fname_full.native(), cv_grabbed_frame_ptr_->image, compression_params);

    cv::displayStatusBar("grabbed frame", "Saved \"" + grabbed_frame_fname_ + "\"");

    if(isMaskValid()) {
        /** save the bounding box */
        cv::Rect tight = tight_bbox_;
        tight.x -= bbox_.x;
        tight.y -= bbox_.y;
        std::ofstream ofs(fname_full.native()+".txt");
        if(!ofs) {
            ROS_ERROR_STREAM("Could not open " << fname_full << ".txt for writing tight bounding box.");
            return;
        }
        ofs << "tight_segmented_" << fname_full.filename().native() << " 1 "
            << tight.x << ' ' << tight.y << ' ' << tight.width << ' ' << tight.height;
        ofs.close();

        /** save the cropped frame */
        fname_full = save_dir_ / ("cropped_" + grabbed_frame_fname_);
        cv::imwrite(fname_full.native(), cropped_frame_, compression_params);
        cv::displayStatusBar("cropped frame", "Saved \"" + fname_full.filename().native() + "\"");

        /** save the segmnted (and cropped) frame */
        fname_full = save_dir_ / ("segmented_" + grabbed_frame_fname_);
        cv::imwrite(fname_full.native(), segmented_frame_, compression_params);
        cv::displayStatusBar("segmented frame", "Saved \"" + fname_full.filename().native() + "\"");

        /** also prepare the tightcrop */
        
        /** save the tight crop */
        fname_full = save_dir_ / ("tight_cropped_" + grabbed_frame_fname_);
        cv::imwrite(fname_full.native(), cropped_frame_(tight), compression_params);
        cv::displayStatusBar("cropped frame", "Saved \"" + fname_full.filename().native() + "\"");

        /** save the tight-cropped segmentation */
        fname_full = save_dir_ / ("tight_segmented_" + grabbed_frame_fname_);
        cv::imwrite(fname_full.native(), segmented_frame_(tight), compression_params);
        cv::displayStatusBar("segmented frame", "Saved \"" + fname_full.filename().native() + "\"");
    }
}

void DatasetCollector::grabbedFrameMouseCb(int event, int x, int y, int flags)
{
    if(!cv_grabbed_frame_ptr_) return;

    switch(mouse_mode_) {
        case MARK_BBOX:
            markBboxMouseCb(event, x, y, flags);
            break;
        case MARK_FG:
        case MARK_BG:
            markMaskMouseCb(event, x, y, flags);
            break;
        default:
            ROS_ERROR("WTF shouldn't get here");
            break;
    }
}

void DatasetCollector::markBboxMouseCb(int event, int x, int y, int flags)
{
    if(cv::EVENT_LBUTTONDOWN == event) {
        top_left_.x = x;
        top_left_.y = y;
        bottom_right_.x = x;
        bottom_right_.y = y;
        new_grabbed_frame_available_ = true;
    } else if(cv::EVENT_MOUSEMOVE == event && (flags&cv::EVENT_FLAG_LBUTTON)) {
        bottom_right_.x = x;
        bottom_right_.y = y;
        new_grabbed_frame_available_ = true;
    } else if(cv::EVENT_LBUTTONUP == event) {
        bottom_right_.x = x;
        bottom_right_.y = y;
        new_grabbed_frame_available_ = true;
        cropCurrentFrame();
        grabCut(cv::GC_INIT_WITH_RECT);
    }
}

void DatasetCollector::markMaskMouseCb(int event, int x, int y, int flags)
{
    CV_Assert(isMaskValid());
    //CV_Assert(0 <= x && x < mask_.cols);
    //CV_Assert(0 <= y && y < mask_.rows);
    CV_Assert(MARK_FG == mouse_mode_ || MARK_BG == mouse_mode_);

    switch(event) {
        case cv::EVENT_MOUSEMOVE:
            if( !( flags & cv::EVENT_FLAG_LBUTTON ) ) {
                mouse_pos_.x = x;
                mouse_pos_.y = y;
                new_grabbed_frame_available_ = true;
                return;
            }
            // fall through
        case cv::EVENT_LBUTTONDOWN:
        case cv::EVENT_LBUTTONUP:
            //mask_/*.getMat(cv::ACCESS_WRITE)*/.at<uint8_t>(y, x) = MARK_FG == mouse_mode_ ? cv::GC_FGD : cv::GC_BGD;
            cv::circle(mask_, cv::Point(x,y), DRAW_WIDTH, MARK_FG == mouse_mode_ ? cv::GC_FGD : cv::GC_BGD, -1, cv::LINE_8, 0);
            new_grabbed_frame_available_ = true;
            break;
    }

    if(cv::EVENT_LBUTTONUP == event) {
        grabCut(cv::GC_INIT_WITH_MASK);
    }

    new_segmented_frame_available_ = true;
}

void DatasetCollector::cropCurrentFrame()
{
    CV_Assert(!!cv_grabbed_frame_ptr_);
    CV_Assert(0 <= top_left_.x && top_left_.x < cv_grabbed_frame_ptr_->image.cols);
    CV_Assert(0 <= top_left_.y && top_left_.y < cv_grabbed_frame_ptr_->image.rows);
    CV_Assert(0 <= bottom_right_.x && bottom_right_.x < cv_grabbed_frame_ptr_->image.cols);
    CV_Assert(0 <= bottom_right_.y && bottom_right_.y < cv_grabbed_frame_ptr_->image.rows);

    bbox_.x = cv::min(top_left_.x, bottom_right_.x);
    bbox_.y = cv::min(top_left_.y, bottom_right_.y);
    bbox_.width  = cv::abs(top_left_.x - bottom_right_.x) + 1;
    bbox_.height = cv::abs(top_left_.y - bottom_right_.y) + 1;

    (cv_grabbed_frame_ptr_->image)(bbox_).copyTo(cropped_frame_);
    new_cropped_frame_available_ = true;
}

void DatasetCollector::markForeground()
{
    if(!isMaskValid()) {
        ROS_ERROR("Invalid mask. Try setting bounding box first.");
        return;
    }
    ROS_INFO("Marking foreground");
    mouse_mode_ = MARK_FG;
}

void DatasetCollector::markBackground()
{
    if(!isMaskValid()) {
        ROS_ERROR("Invalid mask. Try setting bounding box first.");
        return;
    }
    ROS_INFO("Marking background");
    mouse_mode_ = MARK_BG;
}

void DatasetCollector::grabCut(int mode)
{
    if(cv::GC_INIT_WITH_RECT == mode && !isBboxValid()) {
        ROS_WARN("Cannot initialize grab cut with rect because bounding box is invalid");
        return;
    }
    if(cv::GC_INIT_WITH_MASK == mode && !isMaskValid()) {
        ROS_WARN("Cannot initialize grab cut with mask because mask is invalid");
        return;
    }
    if(cv::GC_EVAL == mode && !isMaskValid()) {
        ROS_WARN("Cannot eval grab cut because mask is invalid");
        return;
    }
    cv::grabCut(cv_grabbed_frame_ptr_->image, mask_, bbox_, background_model_, foreground_model_,
            1, mode);
    new_segmented_frame_available_ = true;
    new_grabbed_frame_available_ = true;
}

bool DatasetCollector::isMaskValid() const
{
    return !!cv_grabbed_frame_ptr_ && !cv_grabbed_frame_ptr_->image.empty()
        && !mask_.empty() && cv_grabbed_frame_ptr_->image.size() == mask_.size()
        && mask_.type() == CV_8UC1;
}

bool DatasetCollector::isBboxValid() const
{
    return !!cv_grabbed_frame_ptr_ && !cv_grabbed_frame_ptr_->image.empty()
        && 0 <= bbox_.x && bbox_.x < cv_grabbed_frame_ptr_->image.cols
        && 0 <= bbox_.y && bbox_.y < cv_current_frame_ptr_->image.rows
        && 0 < bbox_.width && 0 < bbox_.height;
}

