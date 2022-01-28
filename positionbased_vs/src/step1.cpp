#include <iostream>
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/robot/vpSimulatorPioneer.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp_bridge/3dpose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TwistStamped.h>
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
  std::cout << "\n" << vel <<std::endl;
  return vel;
}

geometry_msgs::TwistStamped nullVelocity() {
  geometry_msgs::TwistStamped vel;

  vel.twist.linear.x=0;
  vel.twist.linear.y=0;
  vel.twist.linear.z=0;
  vel.twist.angular.x=0;
  vel.twist.angular.y=0;
  vel.twist.angular.z=0;
  return vel;
}

int main(int argc, char** argv)
{ 
  ros::init(argc, argv, "PositionBasedVisualServoing");
  ros::NodeHandle node_handle; 
  ros::AsyncSpinner spinner(1); 
  spinner.start();

  ros::Rate loop_rate(1000);
  ros::Publisher velocityInput = node_handle.advertise<geometry_msgs::TwistStamped>("/servo_server/delta_twist_cmds", 100);

  static const std::string PLANNING_GROUP = "edo";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  try {
    
    // Definition of the reference frames 

    //ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    //ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
    
    //vpHomogeneousMatrix matrix;
    //matrix=homogeneousTransformation("edo_link_2", "edo_link_4");

    //Defines frames
    double translZ=1.1;
    vpHomogeneousMatrix cam_desTtarget;
    cam_desTtarget[2][3] = translZ;

    vpHomogeneousMatrix eeTcam, worldTee, worldTcam; //since they belong to different trees
    worldTee=homogeneousTransformation("edo_base_link", "edo_gripper_link_ee");
    std::cout << "worldTee: \n" << worldTee << std::endl;
    //eeTcam=homogeneousTransformation("edo_gripper_link_ee", "camera_color_optical_frame");
    eeTcam[0][0]=1; eeTcam[0][1]=0; eeTcam[0][2]=0; eeTcam[0][3]=-0.04;
    eeTcam[1][0]=0; eeTcam[1][1]=1; eeTcam[1][2]=0; eeTcam[1][3]=0;
    eeTcam[2][0]=0; eeTcam[2][1]=0; eeTcam[2][2]=1; eeTcam[2][3]=-0.11; 

    worldTcam=worldTee*eeTcam;
    worldTcam=homogeneousTransformation("world", "camera_color_optical_frame"); 
    std::cout << "worldTcam: \n" << worldTcam << std::endl;

    vpHomogeneousMatrix camTtarget;
    camTtarget=homogeneousTransformation("camera_color_optical_frame", "handeye_target"); //initial position camera wrt target
    std::cout << "camTtarget: \n" << camTtarget << std::endl;

    vpHomogeneousMatrix worldTtarget;
    worldTtarget=worldTcam*camTtarget;
    //should be the same worldTtarget=homogeneousTransformation("world", "handeye_target");
    std::cout << "worldTtarget: \n" << worldTtarget << std::endl;
    
    //same object and target frame
    vpPoint point;
    point.setWorldCoordinates(0,0,0); 
    point.track(camTtarget); //object tracking
    std::cout << point << std::endl;

    //Task details
    vpServo task;
    task.setServo(vpServo::EYEINHAND_CAMERA); //control law
    task.setInteractionMatrixType(vpServo::CURRENT); //interaction matrix $\bf L$ is computed from the visual features at the desired position
    task.setLambda(1.4);
    //std::cout << task << std::endl

    vpFeaturePoint s_x, s_xd;
    vpFeatureBuilder::create(s_x, point);
    s_xd.buildFrom(0, 0, translZ);
    //task.addFeature(s_x, s_xd, vpFeaturePoint::selectX()); //not needed

    vpFeatureDepth s_Z, s_Zd;
    double Z = point.get_Z();
    double Zd = translZ;
    s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z, log(Z/Zd)); //initialize depthFeature
    s_Zd.buildFrom(0, 0, Zd, 0);

    task.addFeature(s_Z, s_Zd);

    geometry_msgs::TwistStamped velocityData;
    double error = 5;
    double threshold=0.0002;
    
    while (error>=threshold && node_handle.ok()) {
      //Update camTtarget
      //move_group_interface.getCurrentPose("edo_gripper_link_ee");
      //worldTcam=homogeneousTransformation("world", "camera_color_optical_frame");
      camTtarget=homogeneousTransformation("camera_color_optical_frame", "handeye_target"); //update position camera wrt target
      //worldTtarget=worldTcam*camTtarget;
      //camTtarget=worldTcam.inverse()*worldTtarget;
      //std::cout << "\n" << camTtarget <<std::endl;

      point.track(camTtarget);
      
      vpFeatureBuilder::create(s_x, point);
      Z = point.get_Z();
      s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z, log(Z / Zd));

      //compute Jacobian and others


      vpColVector v = task.computeControlLaw();
      error = (task.getError()).sumSquare(); // error = s^2 - s_star^2
      //std::cout << "\n" << v <<std::endl;

      //send joint commands
      velocityData=insertData(v);

      if (error>=threshold) {
        velocityInput.publish(velocityData);
      }

      else {
        velocityData=nullVelocity();
        velocityInput.publish(velocityData);
        std::cout << "\n" << camTtarget <<std::endl;
      }

      ros::spinOnce();
      loop_rate.sleep();

    }

    return 0;


  }

  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }

}

