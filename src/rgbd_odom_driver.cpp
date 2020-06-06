/*
 * rgbd_odom_driver.cpp.
 *
 * Written by: Stylianos Piperakis.
 *
 * This file launches the rgbd odometry module inspired by DVO.
 */
#include <rgbd_odom_ros/rgbd_odom_ros.h>


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "rgbd_odom_ros");
    ros::NodeHandle n;
    rgbd_odom_ros rgbd_odom(n);
    ros::NodeHandle n_p("~");
    double image_freq;
    n_p.param<double>("image_freq", image_freq, 100.0);
    static ros::Rate rate(2.0*image_freq);
    while (ros::ok())
    {
        rgbd_odom.run();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}