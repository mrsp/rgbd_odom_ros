/* 
 * Copyright 2020 Stylianos Piperakis, Foundation for Research and Technology Hellas (FORTH)
 * License: BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Foundation for Research and Technology Hellas (FORTH) 
 *		 nor the names of its contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @brief RGBD Odometry with ROS
 */
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/rgbd.hpp"
#include <eigen3/Eigen/Dense>
#include <fstream>
using namespace std;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

class rgbd_odom
{
    ///current image frame
    int frame;
    ///image dimensions
    int height, width;
    ///camera distorsion
    double k1, k2, k3, t1, t2;
    ///camera calibration
    double cx, cy, fx, fy;
    /// ROS nodehanlder
    ros::NodeHandle nh;
    /// ROS image transport for image callback
    image_transport::ImageTransport it;
    /// ROS image subscriber only called when RGB image is used and not DEPTH image.
    image_transport::Subscriber image_sub_;
    /// Flags for first Image Callback, first Camera Info Callback, and for new image callback
    bool firstImageCb, firstCameraInfoCb, img_inc;
    // Flags for  checking VO initialization
    bool  voInitialized, mm_to_meters ;
    ///placeholders for previous and current Grayscale/RGB/Depth Image
    cv::Mat currImage, prevImage, currImageRGB, prevDepthImage, currDepthImage;
    cv::Mat  R,  t, cam_intrinsics; //
    ///Eigen 3D Rotation of Previous Image to Current Image Computed with Teaser (when 3D Estimation is ran)
    Eigen::MatrixXd Rot_eig, R_f;
    ///Eigen 3D Translation of Previous Image to Current Image Computed with Teaser (when 3D Estimation is ran)
    Eigen::VectorXd t_eig, t_f;


    ///ROS RGB Image Subscriber
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    ///ROS DEPTH Image Subscriber
    message_filters::Subscriber<sensor_msgs::Image> depth_sub;
    /// ROS Synchronization for RGB and DEPTH msgs
    message_filters::Synchronizer<MySyncPolicy> *ts_sync;
    /// ROS image, depth and camera info topics
    std::string image_topic, depth_topic, cam_info_topic;
    //odometry path from visualization on rviz
    nav_msgs::Path odomPath;
    //odometry path publisher
    ros::Publisher odom_path_pub;
    //current pose from vo
    Eigen::Affine3d curr_pose;
public:
    /** @fn  rgbd_odom(ros::NodeHandle nh_);
	 *  @brief Initializes the VO Benchmarking
     *  @param nh_ ros nodehandler 
	 */
    rgbd_odom(ros::NodeHandle nh_);
    /** @fn is3D()
	 *  @brief return true if klt_ros finds 3d tfs.
	 */
    
  
    void imageDepthCb(const sensor_msgs::ImageConstPtr &img_msg, const sensor_msgs::ImageConstPtr &depth_msg);
    /** @fn void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg);
     * @brief Camera Info Callback
     */
    void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg);
    /** @fn run()
     * @brief computes visual odometry
     */
    void run();
  
    void publishOdomPath();
    /** @fn void addTfToPath(const Eigen::Matrix4d &R_f, const Eigen::VectorXd &t_f)
     *  @param Matrix4d new tf     
     *  @brief add roatiaion R_f and translation t_f to odometry path for publishing later.
     */
    void addTfToPath(const Eigen::Affine3d &pose);
    /** @fn void computeTransformedKeypointsError(std::vector<cv::KeyPoint> matched_currKeypoints, std::vector<cv::KeyPoint> matched_prevKeypoints_transformed);
     *  @brief computes the pixel error of 2D detected Keypoints from current Image and 2D transformed Keypoints from previous Image
     */
};
