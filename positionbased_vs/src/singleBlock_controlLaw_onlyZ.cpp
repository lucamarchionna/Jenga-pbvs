#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp_ros/vpROSGrabber.h>
#include <visp3/io/vpImageStorageWorker.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>


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
  
  //Configuration for the camera
  vpImage<vpRGBa> I; // Create a color image container
  vpROSGrabber gc; // Create a grabber based on ROS
  gc.setImageTopic("/camera/color/image_raw");
  gc.setCameraInfoTopic("/camera/color/camera_info");
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

  vpMe me;
  me.setMaskSize(4);
  me.setMaskNumber(180);
  me.setRange(8);
  me.setThreshold(10000);
  me.setMu1(0.5);
  me.setMu2(0.5);
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
  task.setLambda(0.4);

  vpPoint point;
  point.setWorldCoordinates(0,0,0); 
  point.track(camTtarget); //object tracking

  vpHomogeneousMatrix cdTtarget;
  cdTtarget[0][3]=0.01;
  cdTtarget[1][3]=0.04;
  cdTtarget[2][3]=0.311;

  double translX=0.09;
  double translY=0.19;
  double translZ=0.3;
  vpFeaturePoint s_x, s_xd;
  vpFeatureBuilder::create(s_x, point);
  s_xd.buildFrom(translX, translY, translZ);
  vpFeatureDepth s_Z, s_Zd;
  double X = point.get_X();
  double Y = point.get_Y();
  double Z = point.get_Z();
  double Xd = translX;
  double Yd = translY;
  double Zd = translZ;
  

  s_Z.buildFrom(X, Y, Z, log(Z/Zd)); //initialize depthFeature
  s_Zd.buildFrom(0, Yd, Zd, 0);

  task.addFeature(s_Z, s_Zd);

  geometry_msgs::TwistStamped velocityData;
  double error = 5;
  double threshold=0.0002;
  vpColVector v = task.computeControlLaw();
  std::cout << v << std::endl;
  while(1) {

    // Feedback control loop
    gc.acquire(I);
    vpDisplay::display(I);
    tracker.track(I);
    tracker.getPose(camTtarget);
    tracker.getCameraParameters(cam);
    tracker.display(I, camTtarget, cam, vpColor::red, 2);
    
    point.track(camTtarget);  
    vpFeatureBuilder::create(s_x, point);
    Z = point.get_Z();
    s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z, log(Z / Zd));

    vpColVector v = task.computeControlLaw();
    error = (task.getError()).sumSquare(); // error = s^2 - s_star^2
    velocityData=insertData(v);   //send joint commands
    velocityInput.publish(velocityData);
    vpDisplay::displayFrame(I, camTtarget, cam, 0.025, vpColor::none, 3);
    vpDisplay::displayText(I, 20, 20, "A click to quit...", vpColor::red);
    vpDisplay::flush(I);

    if (vpDisplay::getClick(I, false))
      break;
  }
  
  vpDisplay::getClick(I);
  delete display;
}
