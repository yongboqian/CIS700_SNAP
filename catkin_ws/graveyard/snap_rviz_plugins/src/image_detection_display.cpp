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

#include <boost/bind.hpp>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRectangle2D.h>
#include <OgreRenderSystem.h>
#include <OgreRenderWindow.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>
#include <OgreTechnique.h>
#include <OgreCamera.h>

#include <tf/transform_listener.h>

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/render_panel.h"
#include "rviz/validate_floats.h"

#include <sensor_msgs/image_encodings.h>

#include "image_detection_display.h"
using namespace rviz;

namespace snap_rviz_plugins
{

ImageDetectionDisplay::ImageDetectionDisplay()
  : ImageDisplayBase()
  , texture_()
  , dets_sub_()
  , dets_tf_filter_( 0 )
  , dets_messages_received_( 0 )
  , dets_topic_property_( 0 )
  , new_dets_( false )
  , force_render_( false )
  , dets_ok_( false )
{
  normalize_property_ = new BoolProperty( "Normalize Range", true,
                                          "If set to true, will try to estimate the range of possible values from the received images.",
                                          this, SLOT( updateNormalizeOptions() ));

  min_property_ = new FloatProperty( "Min Value", 0.0, "Value which will be displayed as black.", this, SLOT( updateNormalizeOptions() ));

  max_property_ = new FloatProperty( "Max Value", 1.0, "Value which will be displayed as white.", this, SLOT( updateNormalizeOptions() ));

  median_buffer_size_property_ = new IntProperty( "Median window", 5, "Window size for median filter used for computin min/max.",
                                                  this, SLOT( updateNormalizeOptions() ) );

  got_float_image_ = false;

  dets_topic_property_ = new RosTopicProperty("Detections Topic", "",
          QString::fromStdString(ros::message_traits::datatype<snap_vision_msgs::DetectionsStamped>()),
          "snap_vision_msgs::DetectionsStamped topic to subscribe to.", this, SLOT( updateTopic() ));
}

void ImageDetectionDisplay::onInitialize()
{
  ImageDisplayBase::onInitialize();
  {
    static uint32_t count = 0;
    std::stringstream ss;
    ss << "ImageDetectionDisplay" << count++;
    img_scene_manager_ = Ogre::Root::getSingleton().createSceneManager(Ogre::ST_GENERIC, ss.str());
  }

  img_scene_node_ = img_scene_manager_->getRootSceneNode()->createChildSceneNode();

  {
    static int count = 0;
    std::stringstream ss;
    ss << "ImageDetectionDisplayObject" << count++;

    screen_rect_ = new Ogre::Rectangle2D(true);
    screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
    screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

    ss << "Material";
    material_ = Ogre::MaterialManager::getSingleton().create( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    material_->setSceneBlending( Ogre::SBT_REPLACE );
    material_->setDepthWriteEnabled(false);
    material_->setReceiveShadows(false);
    material_->setDepthCheckEnabled(false);

    material_->getTechnique(0)->setLightingEnabled(false);
    Ogre::TextureUnitState* tu = material_->getTechnique(0)->getPass(0)->createTextureUnitState();
    tu->setTextureName(texture_.getTexture()->getName());
    tu->setTextureFiltering( Ogre::TFO_NONE );

    material_->setCullingMode(Ogre::CULL_NONE);
    Ogre::AxisAlignedBox aabInf;
    aabInf.setInfinite();
    screen_rect_->setBoundingBox(aabInf);
    screen_rect_->setMaterial(material_->getName());
    img_scene_node_->attachObject(screen_rect_);
  }

  render_panel_ = new RenderPanel();
  render_panel_->getRenderWindow()->setAutoUpdated(false);
  render_panel_->getRenderWindow()->setActive( false );

  render_panel_->resize( 640, 480 );
  render_panel_->initialize(img_scene_manager_, context_);

  setAssociatedWidget( render_panel_ );

  render_panel_->setAutoRender(false);
  render_panel_->setOverlaysEnabled(false);
  render_panel_->getCamera()->setNearClipDistance( 0.01f );

  updateNormalizeOptions();

  dets_tf_filter_ = new tf::MessageFilter<snap_vision_msgs::DetectionsStamped>(
          *context_->getTFClient(), fixed_frame_.toStdString(),
          queue_size_property_->getInt(), update_nh_ );
  dets_tf_filter_->connectInput(dets_sub_);
  dets_tf_filter_->registerCallback(boost::bind(&ImageDetectionDisplay::detsCallback, this, _1));
}

ImageDetectionDisplay::~ImageDetectionDisplay()
{
  if ( initialized() )
  {
    // should we delete dets_topic_property_? topic_property_ isn't deleted anywhere
    dets_tf_filter_->clear();
    delete dets_tf_filter_;
    delete render_panel_;
    delete screen_rect_;
    img_scene_node_->getParentSceneNode()->removeAndDestroyChild( img_scene_node_->getName() );
  }
}

void ImageDetectionDisplay::onEnable()
{
  subscribe();
  render_panel_->getRenderWindow()->setActive(true);
}

void ImageDetectionDisplay::onDisable()
{
  render_panel_->getRenderWindow()->setActive(false);
  unsubscribe();
  clear();
}

void ImageDetectionDisplay::subscribe()
{
  if (!isEnabled() || (dets_topic_property_->getTopicStd().empty()))
  {
    return;
  }

  ImageDisplayBase::subscribe();

  std::string dets_topic = dets_topic_property_->getTopicStd();

  try
  {
    dets_sub_.subscribe( update_nh_, dets_topic, (uint32_t)queue_size_property_->getInt());
    setStatus( StatusProperty::Ok, "Detections Stamped", "OK" );
  }
  catch( ros::Exception &e )
  {
    setStatus( StatusProperty::Error, "Detections Stamped", QString( "Error subscribing: ") + e.what() );
  }
}

void ImageDetectionDisplay::unsubscribe()
{
  ImageDisplayBase::unsubscribe();
  dets_sub_.unsubscribe();
}

void ImageDetectionDisplay::updateNormalizeOptions()
{
  if (got_float_image_)
  {
    bool normalize = normalize_property_->getBool();

    normalize_property_->setHidden(false);
    min_property_->setHidden(normalize);
    max_property_->setHidden(normalize);
    median_buffer_size_property_->setHidden(!normalize);

    texture_.setNormalizeFloatImage( normalize, min_property_->getFloat(), max_property_->getFloat());
    texture_.setMedianFrames( median_buffer_size_property_->getInt() );
  }
  else
  {
    normalize_property_->setHidden(true);
    min_property_->setHidden(true);
    max_property_->setHidden(true);
    median_buffer_size_property_->setHidden(true);
  }
}

void ImageDetectionDisplay::clear()
{
  texture_.clear();

  if( render_panel_->getCamera() )
  {
    render_panel_->getCamera()->setPosition(Ogre::Vector3(999999, 999999, 999999));
  }
}

void ImageDetectionDisplay::update( float wall_dt, float ros_dt )
{
  try
  {
    texture_.update();

    //make sure the aspect ratio of the image is preserved
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    float img_width = texture_.getWidth();
    float img_height = texture_.getHeight();

    if ( img_width != 0 && img_height != 0 && win_width !=0 && win_height != 0 )
    {
      float img_aspect = img_width / img_height;
      float win_aspect = win_width / win_height;

      if ( img_aspect > win_aspect )
      {
        screen_rect_->setCorners(-1.0f, 1.0f * win_aspect/img_aspect, 1.0f, -1.0f * win_aspect/img_aspect, false);
      }
      else
      {
        screen_rect_->setCorners(-1.0f * img_aspect/win_aspect, 1.0f, 1.0f * img_aspect/win_aspect, -1.0f, false);
      }
    }

    render_panel_->getRenderWindow()->update();
  }
  catch( UnsupportedImageEncoding& e )
  {
    setStatus(StatusProperty::Error, "Image Detections", e.what());
  }
}

void ImageDetectionDisplay::reset()
{
  ImageDisplayBase::reset();
  clear();
}

/* This is called by incomingMessage(). */
void ImageDetectionDisplay::processMessage(const sensor_msgs::Image::ConstPtr& msg)
{
  bool got_float_image = msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
      msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      msg->encoding == sensor_msgs::image_encodings::TYPE_16SC1 ||
      msg->encoding == sensor_msgs::image_encodings::MONO16;

  if ( got_float_image != got_float_image_ )
  {
    got_float_image_ = got_float_image;
    updateNormalizeOptions();
  }
  texture_.addMessage(msg);
}

} // namespace snap_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( snap_rviz_plugins::ImageDetectionDisplay, rviz::Display )
