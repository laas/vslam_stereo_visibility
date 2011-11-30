#include <stdexcept>

#include <boost/bind.hpp>

#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <VisionLocalization.h>

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

protected:
  void imageCallback (const sensor_msgs::ImageConstPtr& leftImage,
		      const sensor_msgs::CameraInfoConstPtr& leftCamera,
		      const sensor_msgs::ImageConstPtr& rightImage,
		      const sensor_msgs::CameraInfoConstPtr& rightCamera);

private:
  /// \brief Main node handle.
  ros::NodeHandle nodeHandle_;

  VisionLocalization localization_;

  message_filters::Subscriber<sensor_msgs::Image> leftImageSub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> leftCameraSub_;
  message_filters::Subscriber<sensor_msgs::Image> rightImageSub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> rightCameraSub_;

  message_filters::Synchronizer<policy_t> sync_;

  ros::Publisher cameraTransformationPub_;

  sensor_msgs::Image leftImage_;
  sensor_msgs::CameraInfo leftCamera_;
  sensor_msgs::Image rightImage_;
  sensor_msgs::CameraInfo rightCamera_;
};

SlamNode::SlamNode ()
  : nodeHandle_ ("slam_node"),
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
    rightCamera_ ()
{
  // Parameters definition.
  std::string cameraTopicPrefix;
  ros::param::param<std::string>("~camera_prefix", cameraTopicPrefix, "");

  // Topic name construction.
  std::string leftImageTopic = cameraTopicPrefix + "/left/image_raw";
  std::string leftCameraTopic = cameraTopicPrefix + "/left/camera_info";
  std::string rightImageTopic = cameraTopicPrefix + "/right/image_raw";
  std::string rightCameraTopic = cameraTopicPrefix + "/right/camera_info";

  // Publishers creation.
  cameraTransformationPub_ =
    nodeHandle_.advertise<geometry_msgs::TransformStamped>
    ("camera_position", 1);

  // Subscribers creation.
  leftImageSub_.subscribe (nodeHandle_, leftImageTopic, 1);
  leftCameraSub_.subscribe (nodeHandle_, leftCameraTopic, 1);
  rightImageSub_.subscribe (nodeHandle_, rightImageTopic, 1);
  rightCameraSub_.subscribe (nodeHandle_, rightCameraTopic, 1);

  // Message filter creation.
  sync_.connectInput (leftImageSub_, leftCameraSub_,
		      rightImageSub_, rightCameraSub_);

  sync_.registerCallback
    (boost::bind
     (&SlamNode::imageCallback, boost::ref (this), _1, _2, _3, _4));

  waitForImage ();
  if (!ros::ok())
    return;
  if (!leftImage_.width || !leftImage_.height
      || !rightImage_.width || !rightImage_.height)
    throw std::runtime_error ("failed to retrieve image");
}

SlamNode::~SlamNode ()
{}

void
SlamNode::waitForImage ()
{
  ros::Rate loop_rate (10);
  while (ros::ok ()
	 && (!leftImage_.width
	     || !leftImage_.height
	     || !rightImage_.width
	     || !rightImage_.height))
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
}

void
SlamNode::spin ()
{
  ros::Rate rate (100);

  geometry_msgs::TransformStamped cameraTransformation;
  cameraTransformation.header.seq = 0;
  cameraTransformation.header.stamp = ros::Time::now ();
  cameraTransformation.header.frame_id = "/slam_world";

  cameraTransformation.child_frame_id = "/camera"; //FIXME:

  while (ros::ok ())
    {
      // Update header.
      ++cameraTransformation.header.seq;
      cameraTransformation.header.stamp = ros::Time::now (); //FIXME:

      // Localize the camera.
      localization_.Localization_Step
	(&leftImage_.data[0], &rightImage_.data[0], 1);

      // Copy the new camera position.
      cameraTransformation.transform.translation.x =
	localization_.Get_X_Pose ();
      cameraTransformation.transform.translation.y =
	localization_.Get_Y_Pose ();
      cameraTransformation.transform.translation.z =
	localization_.Get_Z_Pose ();

      cameraTransformation.transform.rotation.x =
	localization_.Get_qX_Pose ();
      cameraTransformation.transform.rotation.y =
	localization_.Get_qY_Pose ();
      cameraTransformation.transform.rotation.z =
	localization_.Get_qZ_Pose ();
      cameraTransformation.transform.rotation.w =
	localization_.Get_q0_Pose ();

      // Publish the new camera position.
      cameraTransformationPub_.publish (cameraTransformation);

      rate.sleep ();
      ros::spinOnce ();
    }
}

void
SlamNode::imageCallback (const sensor_msgs::ImageConstPtr& leftImage,
			 const sensor_msgs::CameraInfoConstPtr& leftCamera,
			 const sensor_msgs::ImageConstPtr& rightImage,
			 const sensor_msgs::CameraInfoConstPtr& rightCamera)
{
  leftImage_ = *leftImage;
  leftCamera_ = *leftCamera;
  rightImage_ = *rightImage;
  rightCamera_ = *rightCamera;
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
