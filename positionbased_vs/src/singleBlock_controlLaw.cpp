#include <iostream>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp_ros/vpROSGrabber.h>
#include <visp3/io/vpImageStorageWorker.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/sensor/vpRealSense.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>
#include <visp_bridge/3dpose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std;

vpHomogeneousMatrix homogeneousTransformation(string link1, string link3) {

  vpHomogeneousMatrix homogeneousMatrix;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  tf::StampedTransform transform;
  ros::Time t = ros::Time(0);
  
  try {
    transformStamped = tfBuffer.lookupTransform(link1, link3, t, ros::Duration(3));
    vpHomogeneousMatrix homogeneousMatrix = visp_bridge::toVispHomogeneousMatrix(transformStamped.transform);
    return homogeneousMatrix;
  }

  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  
  }
  return homogeneousMatrix;
}


geometry_msgs::TwistStamped insertData(vpColVector data) {
  geometry_msgs::TwistStamped vel;
  vel.header.stamp = ros::Time::now();
  vel.twist.linear.x=data[0];
  vel.twist.linear.y=data[1];
  vel.twist.linear.z=data[2];
  vel.twist.angular.x=data[3];
  vel.twist.angular.y=data[4];
  vel.twist.angular.z=data[5];
  return vel;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "PBVS_JengaModel");
  ros::NodeHandle node_handle; 
  ros::AsyncSpinner spinner(1); 
  spinner.start();
  ros::Publisher velocityInput = node_handle.advertise<geometry_msgs::TwistStamped>("/velocity_ctrl", 100);

  static const std::string PLANNING_GROUP = "edo";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  //Configuration for the camera
  vpImage<vpRGBa> I; // Create a color image container
  vpROSGrabber gc; // Create a grabber based on ROS
  //gc.setMasterURI("http://10.42.0.49:11311");
  gc.setImageTopic("/camera/color/image_raw");
  gc.setCameraInfoTopic("/camera/depth/camera_info");
  gc.setRectify(true);
  gc.open(I);
  vpDisplayX d(I);  //display images
  vpDisplay *display = NULL;
  display = new vpDisplayX;
  display->init(I, 100, 100, "Model-based tracker");

  //vpImage<unsigned char> I_im; //Input one
  vpCameraParameters cam; //Input two
  vpMatrix K_rs(3,3);
  K_rs[0][0]=914.63; K_rs[0][1]=0.0; K_rs[0][2]=651.46; 
  K_rs[1][0]=0.0; K_rs[1][1]=915.25; K_rs[1][2]=372.35; 
  K_rs[2][0]=0.0; K_rs[2][1]=0.0; K_rs[2][2]=1.0; 
  
  vpHomogeneousMatrix camTtarget; //Output

  //motion (longer distances)
  vpMe me;
  //me.setMaskSize(4);
  //me.setMaskNumber(1800);
  //me.setRange(16);
  //me.setThreshold(10000);
  //me.setMu1(0.4);
  //me.setMu2(0.6);
  //me.setSampleStep(4);

  //vicino
  me.setMaskSize(4);
  me.setMaskNumber(180);
  me.setRange(8);
  me.setThreshold(10000);
  me.setMu1(0.4);
  me.setMu2(0.6);
  me.setSampleStep(2); 
  
  //Configuration for the tracker
  vpMbGenericTracker tracker;
  tracker.setTrackerType(vpMbGenericTracker::EDGE_TRACKER);
  tracker.setMovingEdge(me);
  cam.initFromCalibrationMatrix(K_rs);
  tracker.setCameraParameters(cam);
  tracker.loadModel("model/teabox.cao");  
  tracker.setDisplayFeatures(true);
  tracker.initClick(I, "model/teabox.init", true);  
  tracker.track(I);
  tracker.getPose(camTtarget);
  //camTtarget[0][0]=1; camTtarget[0][1]=0; camTtarget[0][2]=0; camTtarget[0][3]=0;
  //camTtarget[1][0]=0; camTtarget[1][1]=1; camTtarget[1][2]=0; camTtarget[1][3]=0;
  //camTtarget[2][0]=0; camTtarget[2][1]=0; camTtarget[2][2]=1; camTtarget[2][3]=0.18; 
  //tracker.initFromPose(I, camTtarget);
  //Initialize the Task 
  vpServo task;
  task.setServo(vpServo::EYEINHAND_CAMERA); //control law
  task.setInteractionMatrixType(vpServo::CURRENT); //interaction matrix $\bf L$ is computed from the visual features at the desired position
  task.setLambda(0.8);

  vpPoint point;
  point.setWorldCoordinates(0,0,0); 
  point.track(camTtarget); //object tracking

  vpHomogeneousMatrix cdTtarget, camTee;
  cdTtarget[0][3]=-0.03;
  cdTtarget[1][3]=0.0;
  cdTtarget[2][3]=0.7;

  vpFeatureTranslation t(vpFeatureTranslation::cMo);
  vpFeatureTranslation t_star(vpFeatureTranslation::cMo);

  t.buildFrom(camTtarget); //initialize depthFeature
  t_star.buildFrom(cdTtarget);

  task.addFeature(t, t_star); //(t, t_star, vpFeatureTranslation::selectTz());

  geometry_msgs::TwistStamped velocityData;
  double error = 5;
  double threshold=0.0002;

  while(1) {

    // Feedback control loop
    gc.acquire(I);
    vpDisplay::display(I);
    tracker.track(I);
    tracker.getPose(camTtarget);
    tracker.getCameraParameters(cam);
    tracker.display(I, camTtarget, cam, vpColor::red, 2);
    
    t.buildFrom(camTtarget);

    // Convert the velocities into end-effector's RF
    vpColVector v_cam = task.computeControlLaw();
    camTee=homogeneousTransformation("edo_gripper_link_ee", "camera_color_optical_frame"); 
    vpColVector v_ee=camTee.getRotationMatrix()*v_cam.rows(1,3);
    velocityData=insertData(v_ee);   //send joint commands
    velocityInput.publish(velocityData);

    error = (task.getError()).sumSquare(); // error = s^2 - s_star^2
    vpColVector e = t.error(t_star); // e = (s-s*)
    
    vpDisplay::displayFrame(I, camTtarget, cam, 0.025, vpColor::none, 3);
    vpDisplay::displayText(I, 20, 20, "A click to quit...", vpColor::red);
    vpDisplay::flush(I);

    if (vpDisplay::getClick(I, false))
      break;
  }
  

  vpDisplay::getClick(I);
  delete display;
}
