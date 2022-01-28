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

vpHomogeneousMatrix homogeneousTransformation(string link1, string link3) 
{

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

geometry_msgs::PoseStamped insertPosData(vpHomogeneousMatrix data) 
{
  geometry_msgs::PoseStamped pos;
  pos.pose.position.x=data[0][3]; pos.pose.position.y=data[1][3]; pos.pose.position.z=data[2][3];
  
  vpQuaternionVector q;
  data.extract(q);
  
  pos.pose.orientation.x=q[0]; pos.pose.orientation.y=q[1]; pos.pose.orientation.z=q[2]; pos.pose.orientation.w=q[3];

  return pos;
}

geometry_msgs::TwistStamped insertData(vpColVector data) 
{
  geometry_msgs::TwistStamped vel;
  vel.header.stamp = ros::Time::now();
  vel.twist.linear.x=data[0]; vel.twist.linear.y=data[1]; vel.twist.linear.z=data[2];
  vel.twist.angular.x=data[3]; vel.twist.angular.y=data[4]; vel.twist.angular.z=data[5];

  return vel;
}

geometry_msgs::TwistStamped nullVelocity() 
{
  geometry_msgs::TwistStamped vel;
  vel.twist.linear.x=0; vel.twist.linear.y=0; vel.twist.linear.z=0;
  vel.twist.angular.x=0; vel.twist.angular.y=0; vel.twist.angular.z=0;

  return vel;
}



int main(int argc, char** argv)
{ 
  ros::init(argc, argv, "PositionBasedVisualServoing");
  ros::NodeHandle node_handle; 
  ros::AsyncSpinner spinner(1); 
  spinner.start();

  ros::Rate loop_rate(30);
  ros::Publisher velocityInput = node_handle.advertise<geometry_msgs::TwistStamped>("/servo_server/delta_twist_cmds", 1);
  ros::Publisher startingPos = node_handle.advertise<geometry_msgs::PoseStamped>("/initialGuestPos", 1);

  static const std::string PLANNING_GROUP = "edo";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
 
  try {
    //Defines frames
    vpHomogeneousMatrix eeTcam, baseTee, eeTtarget, base2target, targetTcam, cam_desTtarget, camTtarget, cdTc, offset, cdTtarget, camTee; //since they belong to different trees
    eeTcam = homogeneousTransformation("edo_gripper_link_ee", "camera_color_optical_frame");
    camTtarget = homogeneousTransformation("camera_color_optical_frame", "handeye_target");
    targetTcam = homogeneousTransformation("handeye_target", "camera_color_optical_frame");
    baseTee = homogeneousTransformation("edo_base_link", "edo_gripper_link_ee");
    eeTtarget = homogeneousTransformation("edo_gripper_link_ee", "handeye_target");
    camTee=homogeneousTransformation("edo_gripper_link_ee", "camera_color_optical_frame"); 

    geometry_msgs::PoseStamped initialGuestPos;
    vpTranslationVector t_off; t_off << 0, -0.4, 0;
    vpRxyzVector rxyz; rxyz << M_PI_2, 0, 0;
    vpRotationMatrix R_off(rxyz);
    offset.buildFrom(t_off, R_off.inverse());

    base2target = baseTee*eeTtarget;  
    initialGuestPos = insertPosData(base2target*offset);
    //startingPos.publish(initialGuestPos);
    //ros::Duration(2.0).sleep();
    
    vpTranslationVector trans_vec;
    double translX = 0.0;
    double translY = 0.0;
    double translZ = 0.2;    
    trans_vec.buildFrom(translX, translY, translZ);
    vpRotationMatrix cdRo{1, 0, 0, 0, 0, 1, 0, -1, 0};
    cdTtarget.buildFrom(trans_vec, cdRo);

    cdTc = cdTtarget*targetTcam;

    vpPoint point;
    point.setWorldCoordinates(0,0,0);

    //Task details
    vpServo task;
    task.setServo(vpServo::EYEINHAND_CAMERA); //control law
    task.setInteractionMatrixType(vpServo::CURRENT); //interaction matrix $\bf L$ is computed from the visual features at the desired position
    task.setLambda(0.8);

    vpFeatureTranslation s(vpFeatureTranslation::cMo);
    vpFeatureTranslation s_star(vpFeatureTranslation::cMo);
    s.buildFrom(camTtarget);
    s_star.buildFrom(cdTtarget);
    task.addFeature(s, s_star, vpFeatureTranslation::selectTx() | vpFeatureTranslation::selectTy());

    vpFeatureThetaU s_tu(vpFeatureThetaU::cdRc);
    vpFeatureThetaU s_star_tu(vpFeatureThetaU::cdRc); //initialized to zero
    s_tu.buildFrom(cdTc);
    task.addFeature(s_tu, s_star_tu);

    std::cout <<  cdTc << std::endl;

    geometry_msgs::TwistStamped velocityData;
    double error = 5;
    double threshold = 0.000002;
    vpColVector v_ee(3), omega_ee(3), v(6);


    while (error>=threshold && node_handle.ok()) 
    {
      //Update estimation target
      camTtarget = homogeneousTransformation("camera_color_optical_frame", "handeye_target");

      cdTc = cdTtarget*camTtarget.inverse();

      s.buildFrom(camTtarget); //linear
      s_tu.buildFrom(cdTc); //angular

      vpColVector v_cam = task.computeControlLaw();
      error = (task.getError()).sumSquare(); // error = s^2 - s_star^2

      //Convert velocities into the end-effector RF
      v_ee = camTee.getRotationMatrix()*v_cam.rows(1,3);
      omega_ee = camTee.getRotationMatrix()*v_cam.rows(4,6);
      v.insert(0, v_ee); v.insert(3, omega_ee);
      velocityData = insertData(v);

      if (error>=threshold) {
        velocityInput.publish(velocityData);
        vpColVector e = s.error(s_star); // e = (s-s*)
      }

      else {
        velocityData=nullVelocity();
        velocityInput.publish(velocityData);
      }
      ros::spinOnce();
      loop_rate.sleep();
    }

    return 0;
  }

  catch (const vpException &e) 
  {
    std::cout << "Catch an exception: " << e << std::endl;
  }

}

