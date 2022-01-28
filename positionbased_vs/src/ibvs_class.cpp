#include <positionbased_vs/pbvs.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServoDisplay.h>

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
  vel.twist.linear.x=data[0]; vel.twist.linear.y=data[1]; vel.twist.linear.z=data[2];
  vel.twist.angular.x=data[3]; vel.twist.angular.y=data[4]; vel.twist.angular.z=data[5];

  return vel;
}

geometry_msgs::TwistStamped pbvs::nullVelocity() 
{
  geometry_msgs::TwistStamped vel;
  vel.twist.linear.x=0; vel.twist.linear.y=0; vel.twist.linear.z=0.08;
  vel.twist.angular.x=0; vel.twist.angular.y=0; vel.twist.angular.z=0.0;

  return vel;
}


pbvs::pbvs(ros::NodeHandle& nh) : node_handle(nh), s(vpFeatureTranslation::cMo), s_star(vpFeatureTranslation::cMo), s_tu(vpFeatureThetaU::cdRc)
{


  velocityInput = node_handle.advertise<geometry_msgs::TwistStamped>("/servo_server/delta_twist_cmds", 1);
  startingPos = node_handle.advertise<geometry_msgs::PoseStamped>("/initialGuestPos", 1);

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
  eeTtarget = homogeneousTransformation("edo_gripper_link_ee", "handeye_target");
  camTee=homogeneousTransformation("edo_gripper_link_ee", "camera_color_optical_frame"); 

  vpTranslationVector t_off; t_off << 0.0, -0.0, -0.25;
  vpRxyzVector rxyz; rxyz << 0, 0, -M_PI_2;
  vpRotationMatrix R_off(rxyz);
  offset.buildFrom(t_off, R_off.inverse());
  
  baseTtarget = baseTee*eeTtarget; 
  initialGuestPos = insertPosData(baseTtarget*offset);
  //startingPos.publish(initialGuestPos);

  //ros::Duration(10).sleep();

  

  vpTranslationVector cdto;
  translX = 0.063; 
  translY = 0.06;
  translZ = 0.25;    

  cdto.buildFrom(translX, translY, translZ);
  vpRotationMatrix cdRo{1, 0, 0, 
  0, 1, 0, 
  0, 0, 1};

  cdTtarget.buildFrom(cdto, cdRo);

  cdTc = cdTtarget*targetTcam;
  
  Iint.init(480, 640, 255);
  Iext.init(480, 640, 255);
  displayInt.init(Iint, 0, 0, "Internal view");
  displayExt.init(Iext, 670, 0, "External view");
  points.push_back(vpPoint(-0.1, -0.1, 0));
  points.push_back(vpPoint(0.1, -0.1, 0));
  points.push_back(vpPoint(0.1, 0.1, 0));
  points.push_back(vpPoint(-0.1, 0.1, 0));

  for (unsigned int i = 0; i < 4; i++) {
    externalview.insert(points[i]);
  }


}


void pbvs::init_servo()
{
  
  camTtarget = targetTcam.inverse(); 
  camTtarget.extract(t_coord);
  point.setWorldCoordinates(t_coord[0], t_coord[1], t_coord[2]);

  cdTtarget.extract(t_des);
  point_des2.setWorldCoordinates(t_des[0], t_des[1], t_des[2]);

  //Task details
  task.setServo(vpServo::EYEINHAND_CAMERA); //control law
  task.setInteractionMatrixType(vpServo::CURRENT); //interaction matrix $\bf L$ is computed from the visual features at the desired position
  
  const vpAdaptiveGain lambda(1.4, 0.6, 30);
  task.setLambda(lambda);

  vpFeatureBuilder::create(s_x, point);
  vpFeatureBuilder::create(s_x_star, point_des2);
  

  //IBVS  
  task.addFeature(s_x, s_x_star);

  error = 5;
  threshold = 0.00006;
  block_axis = false;
  vitesse = 0.0002;
  rapport = 0;
}

void pbvs::estimationCallback(const geometry_msgs::Pose& tracker_pose_P) 
{

  try
  {
    vpColVector v_ee(3), omega_ee(3), v(6), e1(6);
    
    
    camTtarget = visp_bridge::toVispHomogeneousMatrix(tracker_pose_P); 
    targetTcam = camTtarget.inverse();
    cdTc = cdTtarget*targetTcam;

    camTtarget.extract(t_coord);
    point.setWorldCoordinates(t_coord[0], t_coord[1], t_coord[2]);
    vpFeatureBuilder::create(s_x, point);


    v_cam = task.computeControlLaw();

    /*//Secondary task
    e1[2] = fabs(vitesse);
    proj_e1 = task.secondaryTask(e1);
    rapport = vitesse / proj_e1[2];
    proj_e1 *= rapport;
    v_cam += proj_e1;
    */
    


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
      block_axis = true;
      velocityData=nullVelocity();
      velocityInput.publish(velocityData);
      }

    cout << "end" << endl;


  }

  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "PositionBasedVisualServoing");
  ros::NodeHandle nh;   
  ros::MultiThreadedSpinner spinner(2);
  //spinner.start();
  ros::Rate loop_rate(30); 

  pbvs positionbased_visual_servoing(nh);
  //ros::spinOnce();
  //loop_rate.sleep();
  spinner.spin();
  
  return 0;
}