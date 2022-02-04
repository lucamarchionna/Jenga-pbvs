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
#include "positionbased_vs/InitialGuess.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <edo_core_msgs/JointControlArray.h>
#include <hardware_interface/joint_state_interface.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpProjectionDisplay.h>


using namespace std;

class pbvs 
{
    public:
        pbvs(ros::NodeHandle& nh);
        ~pbvs(); //destroy when class ends
        int init_matrices();
        void init_servo();
        void stopping_criteria();
        void update(const ros::TimerEvent& e);
        void estimationCallback(const geometry_msgs::Pose& tracker_pose_P);
        
        vpHomogeneousMatrix homogeneousTransformation(string link1, string link3);
        geometry_msgs::PoseStamped insertPosData(vpHomogeneousMatrix data);
        geometry_msgs::TwistStamped insertData(vpColVector data);
        geometry_msgs::TwistStamped nullVelocity();
        vpAdaptiveGain lambda(double gain_at_zero, double gain_at_infinity, double slope_at_zero);
        
        

        
    private:
        ros::NodeHandle node_handle;
        //ros::Rate loop_rate;

        ros::Subscriber subEstimationPose; 
        ros::Publisher velocityInput;
        ros::Publisher startingPos;
        ros::Publisher lastPose;
        ros::ServiceClient client;
        
        static const string PLANNING_GROUP; 
        moveit::planning_interface::MoveGroupInterface move_group_interface(string PLANNING_GROUP);
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tf2_ros::Buffer tfBuffer);
        geometry_msgs::TransformStamped transformStamped;
        tf::StampedTransform transform;
        ros::Time t;


        vpHomogeneousMatrix eeTcam, baseTee, eeTtarget, baseTtarget, targetTcam, cam_desTtarget, camTtarget, cdTc, offset, cdTtarget, camTee, camTbase;
        geometry_msgs::PoseStamped initialGuestPos;
        geometry_msgs::Pose lastPoseReceived;
        vpTranslationVector t_off;
        vpRxyzVector rxyz(vpRotationMatrix);
        vpRotationMatrix R_off;
        vpTranslationVector trans_vec;
        double translX;
        double translY;
        double translZ;
        float signPoseReceived{1.0};
        vpRotationMatrix cdRo;


        vpServo task;
        vpFeatureTranslation s;
        vpFeatureTranslation s_star;
        vpFeatureThetaU s_tu;
        vpFeatureThetaU s_tu_star;
        vpPoint point, point_des2;
        vpFeaturePoint ps, point_des;
        vpFeaturePoint p[4], pd[4];
        std::vector<vpPoint> points;
        vpTranslationVector t_coord, t_des;
        vpFeaturePoint s_x, s_x_star;
        
        vpImage<unsigned char> Iint;
        vpImage<unsigned char> Iext;
        vpDisplayX displayInt, displayExt;
        vpProjectionDisplay externalview;
        

        hardware_interface::JointStateInterface joint_state_interface_;
        geometry_msgs::TwistStamped velocityData;
        double error;
        positionbased_vs::InitialGuess srv;
        double threshold, threshold_pose{0.10};
        double vitesse;
        double rapport;
        double tol{0.01};
        bool block_axis{false}, take_cTo{true}, retract{false};
        vpColVector v_ee(unsigned int n), omega_ee(unsigned int n), v_cam, v(unsigned int n), e1, proj_e1;


        edo_core_msgs::JointControlArray jnt_ctrl;
        //const vpException &e;
        
        
        ros::Publisher pub;
        
};
