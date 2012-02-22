#include <stdexcept>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/utility.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>


#define private public
#include <VisionLocalization.h>
#undef private

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

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

  void updateSlamPrior ();
  void localizeCamera ();
  void publishMapFrame ();

  void updateFeatures ();
  void updateFeaturesLoop ();

  void updateDiagnostics (diagnostic_updater::DiagnosticStatusWrapper& stat);

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

  sensor_msgs::Image leftImage_;
  sensor_msgs::CameraInfo leftCamera_;
  sensor_msgs::Image rightImage_;
  sensor_msgs::CameraInfo rightCamera_;

  ros::Publisher featuresPub_;
  boost::thread featuresThread_;

  std::string slamFrameId_;
  tf::TransformBroadcaster tfBroadcaster_;
  tf::TransformListener tfListener_;
  btTransform slamMcamera_;
  tf::StampedTransform cameraMslam_;
  PointCloud features_;

  std::string priorFrameId_;
  /// \brief Given by prior frame.
  tf::StampedTransform slamMcameraEstimated_;

  diagnostic_updater::Updater updater_;
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
    leftImage_ (),
    leftCamera_ (),
    rightImage_ (),
    rightCamera_ (),
    featuresPub_ (),
    featuresThread_ (),
    slamFrameId_ (),
    tfBroadcaster_ (),
    tfListener_ (),
    slamMcamera_ (),
    cameraMslam_ (),
    features_ (),
    priorFrameId_ (),
    slamMcameraEstimated_ (),
    updater_ ()
{
  // Parameters definition.
  std::string cameraTopicPrefix;
  ros::param::param<std::string>("~camera_prefix", cameraTopicPrefix, "");
  ros::param::param<std::string>("~slam_frame_id", slamFrameId_, "/slam");

  ros::param::param<std::string> ("~prior_frame_id", priorFrameId_, "");

  // Topic name construction.
  std::string leftImageTopic = cameraTopicPrefix + "/left/image_mono";
  std::string leftCameraTopic = cameraTopicPrefix + "/left/camera_info";
  std::string rightImageTopic = cameraTopicPrefix + "/right/image_mono";
  std::string rightCameraTopic = cameraTopicPrefix + "/right/camera_info";

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

  // Initialize features.
  featuresPub_ = nodeHandle_.advertise<PointCloud> ("features", 1);
  featuresThread_ = boost::thread (&SlamNode::updateFeaturesLoop, this);

  // Initialize transformation.
  cameraMslam_.child_frame_id_ = slamFrameId_;
  cameraMslam_.frame_id_ = leftImage_.header.frame_id;
  cameraMslam_.getOrigin ().setZero ();

  if (!ros::ok())
    return;
  if (!imageInitialized ())
    throw std::runtime_error ("failed to retrieve image");

  // Updater.
  updater_.setHardwareID ("none");
  updater_.add ("slam", this, &SlamNode::updateDiagnostics);
}

SlamNode::~SlamNode ()
{
  featuresThread_.join ();
}

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
SlamNode::updateSlamPrior ()
{
  if (priorFrameId_.empty ())
    return;

  try
    {
      // camera position w.r.t. SLAM frame (estimated)
      tfListener_.lookupTransform
	(priorFrameId_, slamFrameId_, //leftImage_.header.frame_id,
  	 leftImage_.header.stamp, slamMcameraEstimated_);
      assert (slamMcameraEstimated_.stamp_ == leftImage_.header.stamp);

      TooN::Vector<3> pos;
      TooN::Vector<4> qori;

      pos[0] = slamMcameraEstimated_.getOrigin ()[0] * 1000.;
      pos[1] = slamMcameraEstimated_.getOrigin ()[1] * 1000.;
      pos[2] = slamMcameraEstimated_.getOrigin ()[2] * 1000.;

      btQuaternion qEstimated = slamMcameraEstimated_.getRotation ();
      qori[0] = qEstimated.getW (); // q0
      qori[1] = qEstimated.getX (); // X
      qori[2] = qEstimated.getY (); // Y
      qori[3] = qEstimated.getZ (); // Z

      ROS_DEBUG_STREAM_THROTTLE
	(1,
	 "prior: " << pos << "\n"
	 << "value:" << localization_.Get_Camera_TPose() << "\n"
	 << "prior angle: " << qori << "\n"
	 << "value angle:" << localization_.Get_Quaternion_qCam());

      localization_.Set_Camera_TPose (pos);
      localization_.Set_Camera_qCam (qori);
    }
  catch (const tf::TransformException& e)
    {
      ROS_DEBUG_STREAM_THROTTLE
	(1.,
	 "failed to retrieve prior frame\n"
	 << e.what ());
    }
}


void
SlamNode::localizeCamera ()
{
  // We already localized the robot on the current image, nothing to do.
  if (cameraMslam_.stamp_ == leftImage_.header.stamp)
    return;

  localization_.Localization_Step
    (&leftImage_.data[0], &rightImage_.data[0], 1);

  if (localization_.Get_Rejected_Pose ())
    ROS_WARN_THROTTLE(1., "rejected pose");
  if (localization_.Get_Tracking_Lost ())
    {
      ROS_WARN_THROTTLE(1., "tracking lost");
      return;
    }

  // Inject computed pose into ROS framework.
  slamMcamera_.getOrigin ()[0] = localization_.Get_X_Pose () / 1000.;
  slamMcamera_.getOrigin ()[1] = localization_.Get_Y_Pose () / 1000.;
  slamMcamera_.getOrigin ()[2] = localization_.Get_Z_Pose () / 1000.;

  // The rotation part is world w.r.t camera, so we transpose it.
  TooN::Matrix<3,3> R = localization_.Get_Rotation_CW ().T ();
  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 3; ++j)
      slamMcamera_.getBasis ()[i][j] = R[i][j];

  // We provide to tf the inversed pose.
  cameraMslam_.stamp_ = leftImage_.header.stamp;
  cameraMslam_.setData(slamMcamera_.inverse ());
}

void
SlamNode::publishMapFrame ()
{
  tfBroadcaster_.sendTransform (cameraMslam_);
}

void
SlamNode::spin ()
{
  ros::Rate rate (100);

  while (ros::ok ())
    {
      updateSlamPrior ();
      localizeCamera ();
      publishMapFrame ();
      updater_.update ();

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

void
SlamNode::updateFeatures ()
{
  std::vector<Feature *>* featuresPtr =
    localization_.Scene.Get_List_of_Features ();
  assert (featuresPtr);
  const std::vector<Feature *>& features = *featuresPtr;

  features_.header.frame_id = slamFrameId_;
  features_.header.stamp = ros::Time::now ();
  features_.height = 1;
  features_.width = features.size ();
  features_.points.reserve (features.size ());
  features_.points.clear ();

  BOOST_FOREACH(Feature* feature, features)
    {
      uint8_t r = 0;
      uint8_t g = 0;
      uint8_t b = 0;
      switch (feature->state)
	{
	case 0:
	  r = g = b = 255;
	  break;
	case 1:
	  r = 255;
	  g = b = 0;
	  break;
	case 2:
	  g = 255;
	  r = b = 0;
	  break;
	case 3:
	  r = 255;
	  g = 165;
	  b = 0;
	  break;
	default:
	  assert (0);
	}

      pcl::PointXYZRGB point;
      uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
      point.x = feature->Get_Xw () / 1000.;
      point.y = feature->Get_Yw () / 1000.;
      point.z = feature->Get_Zw () / 1000.;
      point.rgb = *reinterpret_cast<float*>(&rgb);
      features_.points.push_back (point);
    }
}

// Do *not* spin here.
void
SlamNode::updateFeaturesLoop ()
{
  ros::Rate rate (1);

  while (ros::ok ())
    {
      updateFeatures ();
      featuresPub_.publish (features_);
      rate.sleep ();
    }
}

void
SlamNode::updateDiagnostics (diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  stat.summary (diagnostic_msgs::DiagnosticStatus::OK, "OK");

  if (localization_.Get_Rejected_Pose ())
    stat.summary (diagnostic_msgs::DiagnosticStatus::WARN, "Rejected pose");
  if (localization_.Get_Tracking_Lost ())
    stat.summary (diagnostic_msgs::DiagnosticStatus::ERROR, "Tracking lost");

  stat.add("Current number of stereo correspondances",
	   localization_.Get_Number_Current_Stereo_Correspondences ());
  stat.add("Previous number of stereo correspondances",
	   localization_.Get_Number_Previous_Stereo_Correspondences ());

  stat.add("Putatives PnP",
	   localization_.Get_Number_Putatives_PnP ());
  stat.add("Putatives VO",
	   localization_.Get_Number_Putatives_VO ());

  stat.add("Inliers PnP",
	   localization_.Get_Number_Inliers_PnP ());
  stat.add("Inliers VO",
	   localization_.Get_Number_Inliers_VO ());

  stat.add("Ratio Inliers PnP",
	   localization_.Get_Ratio_Inliers_PnP ());
  stat.add("Ratio Inliers VO",
	   localization_.Get_Ratio_Inliers_VO ());

  stat.add("Rejected pose",
	   localization_.Get_Rejected_Pose ());
  stat.add("Rejected VO",
	   localization_.Get_Rejected_VO ());
  stat.add("Tracking lost",
	   localization_.Get_Tracking_Lost ());

  stat.add("Pose (X)", localization_.Get_X_Pose ());
  stat.add("Pose (Y)", localization_.Get_Y_Pose ());
  stat.add("Pose (Z)", localization_.Get_Z_Pose ());

  stat.add("Pose (q0)", localization_.Get_q0_Pose ());
  stat.add("Pose (qX)", localization_.Get_qX_Pose ());
  stat.add("Pose (qY)", localization_.Get_qY_Pose ());
  stat.add("Pose (qZ)", localization_.Get_qZ_Pose ());

  stat.add("cameraMslam (X)", cameraMslam_.getOrigin ()[0]);
  stat.add("cameraMslam (Y)", cameraMslam_.getOrigin ()[1]);
  stat.add("cameraMslam (Z)", cameraMslam_.getOrigin ()[2]);

  stat.add("cameraMslam (qX)", cameraMslam_.getRotation ().getX ());
  stat.add("cameraMslam (qY)", cameraMslam_.getRotation ().getY ());
  stat.add("cameraMslam (qZ)", cameraMslam_.getRotation ().getZ ());
  stat.add("cameraMslam (qW)", cameraMslam_.getRotation ().getW ());

  stat.add("slamMcamera (X)", slamMcamera_.getOrigin ()[0]);
  stat.add("slamMcamera (Y)", slamMcamera_.getOrigin ()[1]);
  stat.add("slamMcamera (Z)", slamMcamera_.getOrigin ()[2]);

  stat.add("slamMcamera (qX)", slamMcamera_.getRotation ().getX ());
  stat.add("slamMcamera (qY)", slamMcamera_.getRotation ().getY ());
  stat.add("slamMcamera (qZ)", slamMcamera_.getRotation ().getZ ());
  stat.add("slamMcamera (qW)", slamMcamera_.getRotation ().getW ());
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
