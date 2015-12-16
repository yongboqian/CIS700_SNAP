/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_IMAGE_DISPLAY_H
#define RVIZ_IMAGE_DISPLAY_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <QObject>

# include <OgreMaterial.h>
# include <OgreRenderTargetListener.h>
# include <OgreSharedPtr.h>

# include "rviz/image/image_display_base.h"
# include "rviz/image/ros_image_texture.h"
# include "rviz/render_panel.h"

# include "rviz/properties/bool_property.h"
# include "rviz/properties/float_property.h"
# include "rviz/properties/int_property.h"

# include <snap_vision_msgs/DetectionsStamped.h>
#endif


namespace Ogre
{
class SceneNode;
class Rectangle2D;
}

namespace snap_rviz_plugins
{

/**
 * \class ImageDetectionDisplay
 *
 */
class ImageDetectionDisplay: public rviz::ImageDisplayBase
{
Q_OBJECT
public:
  ImageDetectionDisplay();
  virtual ~ImageDetectionDisplay();

  // Overrides from Display
  virtual void onInitialize();
  virtual void update( float wall_dt, float ros_dt );
  virtual void reset();

public Q_SLOTS:
  virtual void updateNormalizeOptions();

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  /** @brief ROS topic management. */
  virtual void subscribe();
  virtual void unsubscribe();

  /* This is called by incomingMessage(). */
  virtual void processMessage(const sensor_msgs::Image::ConstPtr& msg);
  virtual void processMessage(const snap_vision_msgs::DetectionsStamped::ConstPtr& msg);

private:
  void clear();
  void updateStatus();

  void detsCallback(const snap_vision_msgs::DetectionsStampedConstPtr &msg);

  Ogre::SceneManager* img_scene_manager_;
  Ogre::SceneNode* img_scene_node_;
  Ogre::Rectangle2D* screen_rect_;
  Ogre::MaterialPtr material_;

  rviz::ROSImageTexture texture_;

  rviz::RenderPanel* render_panel_;

  rviz::BoolProperty* normalize_property_;
  rviz::FloatProperty* min_property_;
  rviz::FloatProperty* max_property_;
  rviz::IntProperty* median_buffer_size_property_;
  bool got_float_image_;

  message_filters::Subscriber<snap_vision_msgs::DetectionsStamped> dets_sub_;
  tf::MessageFilter<snap_vision_msgs::DetectionsStamped> *dets_tf_filter_;
  uint32_t dets_messages_received_;
  rviz::RosTopicProperty *dets_topic_property_;
  std::map<std::string, QColor> dets_colors_;
  bool new_dets_;
  bool force_render_;
  bool dets_ok_;
};

} // namespace snap_rviz_plugins

#endif
