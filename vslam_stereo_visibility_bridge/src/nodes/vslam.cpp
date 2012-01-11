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
    tfListener_ (),
    tfBroadcaster_ ()
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
  leftImageSub_.subscribe (imageTransport_, leftImageTopic, 1);
  leftCameraSub_.subscribe (nodeHandle_, leftCameraTopic, 1);
  rightImageSub_.subscribe (imageTransport_, rightImageTopic, 1);
  rightCameraSub_.subscribe (nodeHandle_, rightCameraTopic, 1);

  // Message filter creation.
  sync_.connectInput (leftImageSub_, leftCameraSub_,
		      rightImageSub_, rightCameraSub_);

  sync_.registerCallback
    (boost::bind
     (&SlamNode::imageCallback, this, _1, _2, _3, _4));

  waitForImage ();
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
SlamNode::spin ()
{
  ros::Rate rate (100);

  geometry_msgs::TransformStamped cameraTransformation;
  cameraTransformation.header.seq = 0;
  cameraTransformation.header.stamp = ros::Time::now ();

  // The world frame is the first position of the left camera when the
  // system starts.
  cameraTransformation.header.frame_id = "/slam_world";

  // The camera position returned by the localization system is the
  // *LEFT* camera position.
  cameraTransformation.child_frame_id = leftCamera_.header.frame_id;

  unsigned lastSeq = leftCamera_.header.seq - 1;

  // camera position w.r.t. slam world
  // We make here the assumption that the slam world is "map".
  tf::StampedTransform mapMc;

  // base link position w.r.t camera
  tf::StampedTransform cMbl;
  // base link position w.r.t. map frame
  tf::StampedTransform mapMbl;


  tf::StampedTransform c0Mc;
  tf::StampedTransform mapMc0
    (tf::Transform (btQuaternion (-0.543, 0.553, -0.455, 0.439),
		    btVector3 (0.106, 0.074, 1.347)),
     ros::Time::now (), "", "");

  ROS_INFO("starting");
  while (ros::ok ())
    {
      // Make sure each image is processed once.
      if (leftCamera_.header.seq > lastSeq)
	{
	  lastSeq = leftCamera_.header.seq;

	  // Update header.
	  ++cameraTransformation.header.seq;
	  cameraTransformation.header.stamp = leftCamera_.header.stamp;

	  // Localize the camera.
	  localization_.Localization_Step
	    (&leftImage_.data[0], &rightImage_.data[0], 1);

	  if(localization_.Get_Tracking_Lost ())
	    ROS_WARN_THROTTLE(1., "tracking lost");
	  if(localization_.Get_Rejected_Pose ())
	    ROS_WARN_THROTTLE(1., "rejected pose");

	  //if(true)
	  if (!localization_.Get_Tracking_Lost ())
	    {
	      // Copy the new camera position, convert to SI.
	      cameraTransformation.transform.translation.x =
		localization_.Get_X_Pose () / 1000.;
	      cameraTransformation.transform.translation.y =
		localization_.Get_Y_Pose () / 1000.;
	      cameraTransformation.transform.translation.z =
		localization_.Get_Z_Pose () / 1000.;

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

	      // Publish to tf.
	      try
		{
		  transformStampedMsgToTF
		    (cameraTransformation, c0Mc);
		  // tfListener_.lookupTransform
		  //   (cameraTransformation.child_frame_id,
		  //    baseLinkFrameId_,
		  //    leftCamera_.header.stamp, cMbl);

		  // Compute base link position w.r.t. to map frame to
		  // localize the robot.
		  mapMc.mult(mapMc0, c0Mc);
		  //mapMbl.mult(mapMc, cMbl);

		  tf::StampedTransform cMmap
		    (mapMc.inverse(), cameraTransformation.header.stamp,
		     cameraTransformation.child_frame_id, mapFrameId_);


		  // If possible, use command to set the roll, pitch
		  // camera orientation and the Z component (height) of
		  // the translation.
		  //
		  // The assumption is that knowing that the robot is
		  // lying on a flat ground the roll, pitch and Z
		  // element cannot drift from their reference
		  // position. However, this neglects the robot
		  // flexibility.
		  try
		    {
		      tf::StampedTransform wMc;
		      tfListener_.lookupTransform
			(cameraTransformation.child_frame_id,
			 "/world", ros::Time(0), wMc);


		      btMatrix3x3& R = cMmap.getBasis();

		      btScalar yaw = 0.;
		      btScalar pitch = 0.;
		      btScalar roll = 0.;
		      R.getEulerYPR(yaw, pitch, roll);

		      btMatrix3x3 R0 = wMc.getBasis();
		      btScalar yaw0 = 0.;
		      btScalar pitch0 = 0.;
		      btScalar roll0 = 0.;
		      R0.getEulerYPR(yaw0, pitch0, roll0);


		      R.setEulerYPR(yaw0, pitch, roll);
		      R[2][3] = R0[2][3];
		    }
		  catch(tf::TransformException ex)
		    {
		      ROS_DEBUG_THROTTLE (1,
					  "failed to retrieve world"
					  " position w.r.t. camera frame.");
		    }

		  tfBroadcaster_.sendTransform(cMmap);
		}
	      catch (tf::TransformException ex)
		{
		  ROS_DEBUG_THROTTLE (1,
				      "failed to retrieve base link"
				      " position w.r.t. camera frame.");
		}
	    }
	  else
	    ROS_WARN_THROTTLE(1., "rejected pose");
	}
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
