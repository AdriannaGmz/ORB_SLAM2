/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>

#include<opencv2/core/core.hpp>
#include "Converter.h"


#include"../../../include/System.h"


//-----
#include "tf/transform_datatypes.h"

 #include "pcl_ros/point_cloud.h"


// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//--


//---
#include <std_msgs/Int16.h>
//---
using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    void PublishPose(cv::Mat Tcw);

    void PublishMap(std::vector<my_point> points);

    void PublishCurrentMap(std::vector<my_point> points);

    void PublishWholeMap(std::vector<my_point> points);

    //void PublishCurrentMatches(void);

    ORB_SLAM2::System* mpSLAM;
    ros::Publisher* pPosPub;

    
    ros::Publisher* pMapPub;

    //TWIST Pos
    ros::Publisher* pPosTwPub;

    //Current Map
    ros::Publisher* pCurrentMapPub;

    //Current Matches
    ros::Publisher* pCurrentMatchesPub;

    //Whole Map
    ros::Publisher* pWholeMapPub;
};

//ros::Publisher pPosPub;

void ImageGrabber::PublishPose(cv::Mat Tcw)
{
    geometry_msgs::PoseStamped poseMSG;
    if(!Tcw.empty())
    {

        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
        
        
        /*
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
            tf::Matrix3x3 M(Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
                            Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
                            Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2));
            tf::Vector3 V(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));

            tf::Transform tfTcw(M,V);

            //mTfBr.sendTransform(tf::StampedTransform(tfTcw,ros::Time::now(), "ORB_SLAM/World", "ORB_SLAM/Camera"));
        */
        //    double tf::getYaw(q);

        tf::Quaternion quat(q[0], q[1], q[2], q[3]);
        tf::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        geometry_msgs::Twist msgOut; 
        msgOut.linear.x = twc.at<float>(2);
        msgOut.linear.y = twc.at<float>(0)*(-1);
        msgOut.linear.z = twc.at<float>(1)*(-1);
        msgOut.angular.x = pitch;
        msgOut.angular.y = roll;
        msgOut.angular.z = yaw;  

        (pPosTwPub)->publish(msgOut);

        poseMSG.pose.position.x = twc.at<float>(2);
        poseMSG.pose.position.y = twc.at<float>(0)*(-1);
        poseMSG.pose.position.z = twc.at<float>(1)*(-1);
        poseMSG.pose.orientation.x = roll;
        poseMSG.pose.orientation.y = pitch;
        poseMSG.pose.orientation.z = yaw;
        poseMSG.pose.orientation.w = q[3];
        poseMSG.header.frame_id = "VSLAM";
        poseMSG.header.stamp = ros::Time::now();
        //cout << "PublishPose position.x = " << poseMSG.pose.position.x << endl;

        (pPosPub)->publish(poseMSG);

        //mlbLost.push_back(mState==LOST);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();
	bool bReuseMap = false;
    if(argc < 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings <false/true>" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
	if (!strcmp(argv[3], "true"))
    {
		bReuseMap = true;
	}
   	ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true, bReuseMap);
    
    //if (bReuseMap)
    //{
    //    SLAM.setMapPath("/home/aldrich/bebop_ws/src/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Slam_latest_Map.bin")
    //    SLAM.LoadMap("/home/aldrich/bebop_ws/src/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Slam_latest_Map.bin");
    //}
	//	SLAM.LoadMap("Slam_latest_Map.bin");
    
	ImageGrabber igb(&SLAM);


    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/bebop/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    // Pose broadcaster
    //pPosPub = new ros::Publisher;
    ros::Publisher PosPub = nodeHandler.advertise<geometry_msgs::PoseStamped>("ORB_SLAM/pose", 5);
   
    igb.pPosPub = &(PosPub);

    //ros::Publisher MapPub = nodeHandler.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("ORB_SLAM/my_Map", 5);
    ros::Publisher MapPub = nodeHandler.advertise<PointCloud>("ORB_SLAM/my_Map", 5);
    
    igb.pMapPub = &(MapPub);

    //Current Map
    ros::Publisher CurrentMapPub = nodeHandler.advertise<PointCloud>("ORB_SLAM/my_CurrentMap", 5);
    
    igb.pCurrentMapPub = &(CurrentMapPub);

    //Twist Pose
    ros::Publisher PosTwPub = nodeHandler.advertise<geometry_msgs::Twist>("ORB2/pose", 5);
    igb.pPosTwPub = &(PosTwPub);
    
    ros::Publisher CurrentMatchesPub = nodeHandler.advertise<std_msgs::Int16>("ORB2/matches", 5);
    igb.pCurrentMatchesPub = &(CurrentMatchesPub);

    ros::Publisher WholeMapPub = nodeHandler.advertise<PointCloud>("ORB_SLAM/my_WholeMap", 5);
    
    igb.pWholeMapPub = &(WholeMapPub);


    ros::spin();

    // Stop all threads
    SLAM.Shutdown();


    // Save map
    SLAM.SaveMap("Slam_latest_Map.bin");
    

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw= mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    PublishPose(Tcw);

    std::vector<my_point> points = mpSLAM->getMap();
    PublishMap(points);

    std::vector<my_point> points2 = mpSLAM->getCurrentMap();
    PublishCurrentMap(points2);

    std::vector<my_point> points3 = mpSLAM->getWholeMap();
    PublishWholeMap(points3);
    //usleep(10000);

    //std::vector<my_point> points2 = mpSLAM->getCurrentMap();
    //PublishCurrentMap(points2);

    //std::vector<my_matches> matches = mpSLAM->getCurrentMatches();
    //PublishCurrentMatches(matches);
    

    std_msgs::Int16 matches;
    //ROS_INFO_STREAM("Current Matches="<<dato);
    matches.data = mpSLAM->my_currentMatches();
    (pCurrentMatchesPub)->publish(matches);
    //ROS_INFO("matches: %d", matches.data);

}

void ImageGrabber::PublishMap(std::vector<my_point> points)
{

   //pcl::PointCloud<pcl::PointXYZRGB> cloudMSG;
   
   PointCloud::Ptr cloudMSG (new PointCloud);
   cloudMSG->header.frame_id = "VSLAM";

   cloudMSG->height = 1;
   cloudMSG->width = points.size();
   //int tam = points.size();
   //ROS_INFO_STREAM("Global points="<<tam);

   //uint8_t r(255), g(15), b(15);

    for(int i=0; i<points.size(); ++i)
    {
    	//cloudMSG->points.push_back (pcl::PointXYZ(points[i].x, points[i].y, points[i].z));
      cloudMSG->points.push_back (pcl::PointXYZ(points[i].z, points[i].x*(-1), points[i].y*(-1)));
    }
    
   (pMapPub)->publish(cloudMSG);

}

void ImageGrabber::PublishCurrentMap(std::vector<my_point> points)
{

   //pcl::PointCloud<pcl::PointXYZRGB> cloudMSG;
   
   PointCloud::Ptr cloudMSG (new PointCloud);
   cloudMSG->header.frame_id = "VSLAM";

   cloudMSG->height = 1;
   cloudMSG->width = points.size();
   
   for(int i=0; i<points.size(); ++i)
   {
    /*
      pcl::PointXYZRGB point;
     
      point.x = points[i].x;
      point.y = points[i].y;
      point.z = points[i].z;
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      point.rgb = *reinterpret_cast<float*>(&rgb);

      cloudMSG.push_back(point);
    */

    cloudMSG->points.push_back (pcl::PointXYZ(points[i].z, points[i].x*(-1), points[i].y*(-1)));
   }
  
   (pCurrentMapPub)->publish(cloudMSG);

}

void ImageGrabber::PublishWholeMap(std::vector<my_point> points)
{

   //pcl::PointCloud<pcl::PointXYZRGB> cloudMSG;
   
   PointCloud::Ptr cloudMSG (new PointCloud);
   cloudMSG->header.frame_id = "VSLAM";

   cloudMSG->height = 1;
   cloudMSG->width = points.size();
   
   for(int i=0; i<points.size(); ++i)
   {
    cloudMSG->points.push_back (pcl::PointXYZ(points[i].z, points[i].x*(-1), points[i].y*(-1)));
   }
  
   (pWholeMapPub)->publish(cloudMSG);

}

/*
void ImageGrabber::PublishCurrentMatches(void)
{
    std_msgs::Int8 msg;
    int dato;
    dato = matches[0].match;
    //ROS_INFO_STREAM("Current Matches="<<dato);
    msg.data = dato;
    (pCurrentMatchesPub)->publish(msg);
}
*/
