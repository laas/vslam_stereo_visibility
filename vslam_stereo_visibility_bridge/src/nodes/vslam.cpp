#include <stdexcept>

#include <boost/bind.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <VisionLocalization.h>

namespace debug
{
  std::ostream& displayPose (std::ostream& o, btTransform& t)
  {
    double y = 0.;
    double p = 0.;
    double r = 0.;
    t.getBasis ().getEulerYPR (y, p, r);
    o << " ";
    o << r;
    o << " ";
    o << p;
    o << " ";
    o << y;
    o << " ";
    o << t.getOrigin () [0];
    o << " ";
    o << t.getOrigin () [1];
    o << " ";
    o << t.getOrigin () [2];
    o << "\n";
    return o;
  }

  std::string getPoseString (btTransform& t)
  {
    std::stringstream ss;
    displayPose(ss, t);
    return ss.str ();
  }
} // end of namespace debug.

class SlamNode
{
public:
  typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::Image, sensor_msgs::CameraInfo,
    sensor_msgs::Image, sensor_msgs::CameraInfo>
    policy_t;

  explicit SlamNode ();
  ~SlamNode ();
  void waitForImage();
  void spin ();

  bool imageInitialized ();

protected:
  void imageCallback (const sensor_msgs::ImageConstPtr& leftImage,
		      const sensor_msgs::CameraInfoConstPtr& leftCamera,
		      const sensor_msgs::ImageConstPtr& rightImage,
		      const sensor_msgs::CameraInfoConstPtr& rightCamera);

  void retrieveCameraPositionUsingControl ();
  void estimateCameraPosition ();
  void localizeCamera ();
  void correctCameraPosition ();
  void publishMapFrame ();

private:
  /// \brief Main node handle.
  ros::NodeHandle nodeHandle_;

  image_transport::ImageTransport imageTransport_;

  VisionLocalization localization_;

  image_transport::SubscriberFilter leftImageSub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> leftCameraSub_;
  image_transport::SubscriberFilter rightImageSub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> rightCameraSub_;

  message_filters::Synchronizer<policy_t> sync_;

  ros::Publisher cameraTransformationPub_;

  sensor_msgs::Image leftImage_;
  sensor_msgs::CameraInfo leftCamera_;
  sensor_msgs::Image rightImage_;
  sensor_msgs::CameraInfo rightCamera_;

  std::string mapFrameId_;
  std::string baseLinkFrameId_;
  tf::TransformListener tfListener_;
  tf::TransformBroadcaster tfBroadcaster_;

  tf::StampedTransform wMcNow_;
  tf::StampedTransform wMcCameraTime_;

  btTransform mapMc0_;

  btTransform c0Mc_;

  tf::StampedTransform cMmap_;
  tf::StampedTransform cMmapCorrected_;
  bool firstTime_;
};

SlamNode::SlamNode ()
  : nodeHandle_ ("slam_node"),
    imageTransport_ (nodeHandle_),
    localization_ (),
    leftImageSub_ (),
    leftCameraSub_ (),
    rightImageSub_ (),
    rightCameraSub_ (),
    sync_ (policy_t(10)),
    cameraTransformationPub_ (),

    leftImage_ (),
    leftCamera_ (),
    rightImage_ (),
    rightCamera_ (),

    mapFrameId_ (),
    baseLinkFrameId_ (),
    tfListener_ (nodeHandle_, ros::Duration (10), true),
    tfBroadcaster_ (),

    wMcNow_ (),
    wMcCameraTime_ (),

    mapMc0_ (),

    c0Mc_ (),

    cMmap_ (),
    cMmapCorrected_ (),

    firstTime_ (true)
{
  // Parameters definition.
  std::string cameraTopicPrefix;
  ros::param::param<std::string>("~camera_prefix", cameraTopicPrefix, "");

  ros::param::param<std::string>("~map_frame_id", mapFrameId_, "/map");
  ros::param::param<std::string>("~base_link_frame_id", baseLinkFrameId_,
				 "/left_ankle"); //FIXME:

  // Topic name construction.
  std::string leftImageTopic = cameraTopicPrefix + "/left/image_mono";
  std::string leftCameraTopic = cameraTopicPrefix + "/left/camera_info";
  std::string rightImageTopic = cameraTopicPrefix + "/right/image_mono";
  std::string rightCameraTopic = cameraTopicPrefix + "/right/camera_info";

  // Publishers creation.
  cameraTransformationPub_ =
    nodeHandle_.advertise<geometry_msgs::TransformStamped>
    ("camera_position", 1);

  // Subscribers creation.
  leftImageSub_.subscribe (imageTransport_, leftImageTopic, 10);
  leftCameraSub_.subscribe (nodeHandle_, leftCameraTopic, 10);
  rightImageSub_.subscribe (imageTransport_, rightImageTopic, 10);
  rightCameraSub_.subscribe (nodeHandle_, rightCameraTopic, 10);

  // Message filter creation.
  sync_.connectInput (leftImageSub_, leftCameraSub_,
		      rightImageSub_, rightCameraSub_);

  sync_.registerCallback
    (boost::bind
     (&SlamNode::imageCallback, this, _1, _2, _3, _4));

  waitForImage ();

  // Initialize tf messages.
  cMmap_.frame_id_ = leftImage_.header.frame_id;
  cMmap_.child_frame_id_ = mapFrameId_;

  if (!ros::ok())
    return;
  if (!imageInitialized ())
    throw std::runtime_error ("failed to retrieve image");
}

SlamNode::~SlamNode ()
{}

void
SlamNode::waitForImage ()
{
  ros::Rate loop_rate (10);
  while (ros::ok () && !imageInitialized ())
    {
      ROS_INFO_THROTTLE
	(5, "waiting for synchronized image and camera data...");
      ros::spinOnce ();
      loop_rate.sleep ();
    }
}

void
SlamNode::retrieveCameraPositionUsingControl ()
{
  static const char* worldFrameId = "/world";

  try
    {
      // camera position w.r.t. world frame
      tfListener_.lookupTransform
	(worldFrameId, leftImage_.header.frame_id,
	 ros::Time (), wMcNow_);
      tfListener_.lookupTransform
	(worldFrameId, leftImage_.header.frame_id,
	 leftImage_.header.stamp, wMcCameraTime_);

      ROS_DEBUG_THROTTLE
	(1.,
	 "camera position w.r.t. world frame sucessfully retrieved");
    }
  catch (const tf::TransformException& e)
    {
      ROS_DEBUG_STREAM_THROTTLE
	(1.,
	 "failed to retrieve camera position w.r.t. world frame\n"
	 << e.what ());
    }
}

void
SlamNode::estimateCameraPosition ()
{
}

void
SlamNode::localizeCamera ()
{
  static const bool controlOnly = false;

  if (controlOnly)
    {
      cMmap_.setData (wMcCameraTime_.inverse ());
      cMmap_.stamp_ = wMcCameraTime_.stamp_;
      return;
    }

  if (firstTime_)
    {
      // We need control data at the first time to initialize the SLAM
      // correctly.
      if (wMcCameraTime_.stamp_ != leftImage_.header.stamp)
	return;
      mapMc0_ = wMcCameraTime_;
      ROS_INFO_STREAM ("initial camera position initialization\n"
		       << debug::getPoseString(mapMc0_));
    }

  // We already localized the robot on the current image, nothing to do.
  if (cMmap_.stamp_ == leftImage_.header.stamp)
    return;

  localization_.Localization_Step
    (&leftImage_.data[0], &rightImage_.data[0], 1);

  //c0Mc_.stamp_ = leftImage_.header.stamp;
  c0Mc_.getOrigin ()[0] = localization_.Get_X_Pose () / 1000.;
  c0Mc_.getOrigin ()[1] = localization_.Get_Y_Pose () / 1000.;
  c0Mc_.getOrigin ()[2] = localization_.Get_Z_Pose () / 1000.;

  btQuaternion q
    (localization_.Get_qX_Pose (),
     localization_.Get_qY_Pose (),
     localization_.Get_qZ_Pose (),
     localization_.Get_q0_Pose ());
  c0Mc_.getBasis () = btMatrix3x3 (q);

  if(localization_.Get_Rejected_Pose ())
    ROS_WARN_THROTTLE(1., "rejected pose");
  if(localization_.Get_Tracking_Lost ())
    {
      ROS_WARN_THROTTLE(1., "tracking lost");
      return;
    }

  if (firstTime_)
    {
      firstTime_ = false;
      ROS_INFO ("first localization went well");
    }

  btTransform c0Mmap = mapMc0_.inverse ();
  btTransform cMc0 = c0Mc_.inverse ();


  cMmap_.setData (cMc0 * c0Mmap);
  cMmap_.stamp_ = wMcCameraTime_.stamp_;
}

void
SlamNode::correctCameraPosition ()
{
  cMmapCorrected_ = cMmap_;
  cMmapCorrected_.frame_id_ = cMmap_.frame_id_;
  cMmapCorrected_.child_frame_id_ = "/map_corrected";

  btTransform cMw = wMcCameraTime_.inverse ();

  cMmapCorrected_.getOrigin ()[1] = cMw.getOrigin ()[1];

  double y = 0.;
  double p = 0.;
  double r = 0.;

  double yCtrl = 0.;
  double pCtrl = 0.;
  double rCtrl = 0.;

  cMmapCorrected_.getBasis ().getEulerYPR (y, p, r);
  cMw.getBasis ().getEulerYPR (yCtrl, pCtrl, rCtrl);

  cMmapCorrected_.getBasis ().setEulerYPR (yCtrl, p, rCtrl);
}

void
SlamNode::publishMapFrame ()
{
  if (firstTime_)
    return;

  //FIXME this is cMmap at camera time, move it to now...
  tfBroadcaster_.sendTransform (cMmap_);
  tfBroadcaster_.sendTransform (cMmapCorrected_);
  //cameraTransformationPub_.publish (cMmapCorrected_);
}

void
SlamNode::spin ()
{
  ros::Rate rate (100);

  while (ros::ok ())
    {
      retrieveCameraPositionUsingControl ();
      localizeCamera ();
      correctCameraPosition ();
      publishMapFrame ();

      ros::spinOnce ();
      rate.sleep ();
    }
}

void
SlamNode::imageCallback (const sensor_msgs::ImageConstPtr& leftImage,
			 const sensor_msgs::CameraInfoConstPtr& leftCamera,
			 const sensor_msgs::ImageConstPtr& rightImage,
			 const sensor_msgs::CameraInfoConstPtr& rightCamera)
{
  if (!leftImage || !leftCamera || !rightImage || !rightCamera)
    {
      ROS_WARN_THROTTLE (1, "null pointer in image callback");
      return;
    }

  if (leftImage->encoding != sensor_msgs::image_encodings::MONO8
      || rightImage->encoding != sensor_msgs::image_encodings::MONO8)
    {
      ROS_WARN_THROTTLE
	(1, "invalid image encoding, should be mono8. Ignoring received images...");
      return;
    }

  leftImage_ = *leftImage;
  leftCamera_ = *leftCamera;
  rightImage_ = *rightImage;
  rightCamera_ = *rightCamera;

  ROS_INFO_THROTTLE
    (5, "image callback called");
}

bool
SlamNode::imageInitialized ()
{
  return leftImage_.width > 0 && leftImage_.height > 0
    && rightImage_.width > 0 && rightImage_.height > 0;
}


int main (int argc, char **argv)
{
  try
    {
      ros::init(argc, argv, "halfsteps_pattern_generator");
      SlamNode node;
      if (ros::ok())
	node.spin();
    }
  catch (std::exception& e)
    {
      std::cerr << "fatal error: " << e.what() << std::endl;
      ROS_ERROR_STREAM("fatal error: " << e.what());
      return 1;
    }
  catch (...)
    {
      ROS_ERROR_STREAM("unexpected error");
      return 2;
    }
  return 0;
}
