#include <positionbased_vs/pbvs.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <visp3/visual_features/vpFeatureBuilder.h>

vpHomogeneousMatrix pbvs::homogeneousTransformation(string link1, string link3) 
{
  vpHomogeneousMatrix homogeneousMatrix;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  t = ros::Time(0);

  try {
    transformStamped = tfBuffer.lookupTransform(link1, link3, t, ros::Duration(3));
    homogeneousMatrix = visp_bridge::toVispHomogeneousMatrix(transformStamped.transform);
  }

  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  
  }

  return homogeneousMatrix;

}

geometry_msgs::PoseStamped pbvs::insertPosData(vpHomogeneousMatrix T_matrix) 
{
  geometry_msgs::PoseStamped to_PS;
  vpQuaternionVector q;
  T_matrix.extract(q);

  to_PS.pose.position.x = T_matrix[0][3];
  to_PS.pose.position.y = T_matrix[1][3]; 
  to_PS.pose.position.z = T_matrix[2][3];
 
  to_PS.pose.orientation.x = q[0]; 
  to_PS.pose.orientation.y = q[1]; 
  to_PS.pose.orientation.z = q[2]; 
  to_PS.pose.orientation.w = q[3];

  return to_PS;
}

geometry_msgs::TwistStamped pbvs::insertData(vpColVector data) 
{
  geometry_msgs::TwistStamped vel;
  vel.header.stamp = ros::Time::now();
  vel.twist.linear.x = data[0]; vel.twist.linear.y = data[1]; vel.twist.linear.z = data[2];
  vel.twist.angular.x = data[3]; vel.twist.angular.y = data[4]; vel.twist.angular.z = 0;

  return vel;
}


pbvs::pbvs(ros::NodeHandle& nh) : node_handle(nh), s(vpFeatureTranslation::cMo), s_star(vpFeatureTranslation::cMo), s_tu(vpFeatureThetaU::cdRc)
{
  client = node_handle.serviceClient<positionbased_vs::InitialGuess>("/InitialGuess");

  velocityInput = node_handle.advertise<geometry_msgs::TwistStamped>("/servo_server/delta_twist_cmds", 1);
  startingPos = node_handle.advertise<geometry_msgs::PoseStamped>("/initialGuestPos", 1);
  lastPose = node_handle.advertise<geometry_msgs::Pose>("/lastPose", 1);

  init_matrices();
  init_servo();

  subEstimationPose = node_handle.subscribe("/pose_estimation", 1, &pbvs::estimationCallback, this);
}

pbvs::~pbvs() {
}

int pbvs::init_matrices() 
{ 
  static const std::string PLANNING_GROUP = "edo";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  eeTcam = homogeneousTransformation("edo_gripper_link_ee", "camera_color_optical_frame");
  camTtarget = homogeneousTransformation("camera_color_optical_frame", "handeye_target");
  targetTcam = homogeneousTransformation("handeye_target", "camera_color_optical_frame");
  baseTee = homogeneousTransformation("edo_base_link", "edo_gripper_link_ee");
  baseTtarget = homogeneousTransformation("edo_base_link", "handeye_target");
  eeTtarget = homogeneousTransformation("edo_gripper_link_ee", "handeye_target");
  camTee=homogeneousTransformation("edo_gripper_link_ee", "camera_color_optical_frame"); 
  camTbase = homogeneousTransformation("camera_color_optical_frame", "edo_base_link");

  vpTranslationVector t_off; t_off << 0.0, -0.0, -0.25;
  vpRxyzVector rxyz; rxyz << 0, 0, -M_PI_2;
  vpRotationMatrix R_off(rxyz);
  offset.buildFrom(t_off, R_off.inverse());
  
  //base2target = baseTee*eeTtarget; 
  //initialGuestPos = insertPosData(base2target*offset);
  //startingPos.publish(initialGuestPos);
  //ros::Duration(10).sleep();

  // bool initialPose{true};

  // ros::service::waitForService("/InitialGuess");
  // srv.request.initialGuessPose = initialPose;
  // ROS_INFO("Calling the service..");

  // if (client.call(srv)){
  //   if (srv.response.execution.data == true) {
  //     ROS_INFO("Approaching the target");
  //   }
  //   else {
  //     ROS_INFO("Initial guess cannot be provided");
  //     return EXIT_FAILURE;
  //   }
  
  // }

  // else {
  //   ROS_ERROR("Cannot receive response from server");
  // }

  vpThetaUVector cTo_tu = camTtarget.getThetaUVector();

  vpTranslationVector cdto;
  translX = 0.0324;  //the correct one is x = -0.0035;
  translY = 0.06;  //the correct one is y = 0.053;
  translZ = 0.27;   //the correct one is z = 0.134

  cdto.buildFrom(translX, translY, translZ);
  vpRotationMatrix cdRo{1, 0, 0, 
  0, 1, 0, 
  0, 0, 1};

  cdTtarget.buildFrom(cdto, cdRo);

  cdTc = cdTtarget*targetTcam;

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped target;

  target.header.stamp = ros::Time::now();
  target.header.frame_id = "camera_desired";
  target.child_frame_id = "handeye_target";
  target.transform=visp_bridge::toGeometryMsgsTransform(cdTtarget); 

  br.sendTransform(target);
 
}


void pbvs::init_servo()
{
  //Task details
  task.setServo(vpServo::EYEINHAND_CAMERA); //control law
  task.setInteractionMatrixType(vpServo::CURRENT); //interaction matrix $\bf L$ is computed from the visual features at the desired position
  
  const vpAdaptiveGain lambda(1.4, 0.6, 30);
  task.setLambda(lambda);

  //s = new vpFeatureTranslation(vpFeatureTranslation::cMo); alternativa con puntatore
  //PBVS
  s.buildFrom(camTtarget);
  s_star.buildFrom(cdTtarget);
  task.addFeature(s, s_star);

  s_tu.buildFrom(cdTc);
  task.addFeature(s_tu, s_tu_star); //, vpFeatureThetaU::selectTUy() | vpFeatureThetaU::selectTUx());

  error = 5;
  // threshold = 0.00002;
  threshold = 0.00008; //only for simulation
  block_axis = false;
  vitesse = 0.0002;
  rapport = 0;
  threshold_pose = 0.015;


}

void pbvs::estimationCallback(const geometry_msgs::Pose& tracker_pose_P) 
{

  vpColVector v_ee(3), omega_ee(3), v(6), e1(6);
  //e1 = 0;

  if (!block_axis) {
    camTtarget = visp_bridge::toVispHomogeneousMatrix(tracker_pose_P); 
    targetTcam = camTtarget.inverse();
    cdTc = cdTtarget*targetTcam;
  }

  else {
    camTbase = homogeneousTransformation("camera_color_optical_frame", "edo_base_link");
    camTtarget = camTbase*baseTtarget;
    targetTcam = camTtarget.inverse();
    cdTc = cdTtarget*targetTcam;
  }

  s.buildFrom(camTtarget); //linear
  s_tu.buildFrom(cdTc); //angular

  v_cam = task.computeControlLaw();

  error = (task.getError()).sumSquare(); // error = s^2 - s_star^2

  //Convert velocities into the end-effector RF
  v_ee = camTee.getRotationMatrix()*v_cam.rows(1,3);
  
  omega_ee = camTee.getRotationMatrix()*v_cam.rows(4,6);
  v.insert(0, v_ee); v.insert(3, omega_ee);
  velocityData = insertData(v);

  if (error>=threshold && !block_axis) {
    
    velocityInput.publish(velocityData);
    vpColVector e = s.error(s_star); // e = (s-s*)

  }

  else {

    if (take_cTo) {
      // baseTtarget = homogeneousTransformation("edo_base_link", "handeye_target");
      // ros::Duration(3).sleep();
      // ROS_INFO("New pose took");
      take_cTo = false;
      lastPose.publish(tracker_pose_P);
      lastPoseReceived = tracker_pose_P;
    }

    else {    
      block_axis = true;
      //velocityData = approach(velocityData);

      if (lastPoseReceived.position.z - camTtarget[2][3] < threshold_pose && retract == false )  {
        signPoseReceived = 1.0; 
        velocityData.twist.linear.z = signPoseReceived*0.04;
        velocityInput.publish(velocityData);
      }

      else if (lastPoseReceived.position.z - camTtarget[2][3] < 0 && retract == true)  {
        signPoseReceived = 0.0; 
      }

      else {
        signPoseReceived = -1.0; 
        velocityData.twist.linear.z = signPoseReceived*0.04;
        velocityInput.publish(velocityData);
        retract = true;
      }


      }
    }



}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "PositionBasedVisualServoing");
  ros::NodeHandle nh;   
  ros::MultiThreadedSpinner spinner(2);
  //spinner.start();
  ros::Rate loop_rate(10); 

  pbvs positionbased_visual_servoing(nh);
  //ros::spinOnce();
  //loop_rate.sleep();
  spinner.spin();
  
  return 0;
}