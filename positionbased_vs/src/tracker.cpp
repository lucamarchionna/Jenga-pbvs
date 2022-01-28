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



#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PBVS_JengaModel");
  ros::NodeHandle node_handle; 
  ros::AsyncSpinner spinner(1); 
  spinner.start();
  


  //Configuration for the camera
  vpImage<vpRGBa> I; // Create a color image container
  vpROSGrabber g; // Create a grabber based on ROS
  g.setImageTopic("/camera/color/image_raw");
  g.setCameraInfoTopic("/camera/color/camera_info");
  g.setRectify(true);
  g.open(I);
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
  
    
  vpHomogeneousMatrix cMo; //Output

  vpMe me;
  me.setMaskSize(5);
  me.setMaskNumber(180);
  me.setRange(8);
  me.setThreshold(5000);
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
  //cMo[0][0]=1; cMo[0][1]=0; cMo[0][2]=0; cMo[0][3]=0;
  //cMo[1][0]=0; cMo[1][1]=1; cMo[1][2]=0; cMo[1][3]=0;
  //cMo[2][0]=0; cMo[2][1]=0; cMo[2][2]=1; cMo[2][3]=0.18; 
  //tracker.initFromPose(I, cMo);

  while(1) {
    //! [Acquisition]
    g.acquire(I);
    vpDisplay::display(I);
    tracker.track(I);
    tracker.getPose(cMo);
    tracker.getCameraParameters(cam);
    tracker.display(I, cMo, cam, vpColor::red, 2);
    vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);
    vpDisplay::displayText(I, 20, 20, "A click to quit...", vpColor::red);
    vpDisplay::flush(I);

    if (vpDisplay::getClick(I, false))
      break;

    //std::cout << cMo << std::endl;
  }
  
  vpDisplay::getClick(I);
  delete display;

}
