#include <rgbd_odom_ros/rgbd_odom.h>




const float MIN_DEPTH = 0.8f;       // in meters
const float MAX_DEPTH = 4.0f;       // in meters
const float MAX_DEPTH_DIFF = 0.08f; // in meters
const float MAX_POINTS_PART = 0.09f;

// This is for TUM dataset
const float PIXEL_TO_METER_SCALEFACTOR = 0.0002;

rgbd_odom::rgbd_odom(ros::NodeHandle nh_) : it(nh_)
{

    nh = nh_;

    img_inc = false;
    firstImageCb = true;
    firstCameraInfoCb = true;
    isFirst = true;
    voInitialized = false;

    frame = 0;
    R_f.resize(3, 3);
    R_f.setIdentity();
    t_f.resize(3);
    t_f.setZero();

    R = cv::Mat::eye(3, 3, CV_64F);
    t = cv::Mat::zeros(3, 1, CV_64F);


    cam_intrinsics = cv::Mat::zeros(3, 3, CV_64F);
    ros::NodeHandle n_p("~");

    curr_pose = Eigen::Affine3d::Identity();
    n_p.param<std::string>("image_topic", image_topic, "/camera/rgb/image_rect_color");
    n_p.param<std::string>("depth_topic", depth_topic, "/camera/depth_registered/sw_registered/image_rect");
    n_p.param<std::string>("cam_info_topic", cam_info_topic, "/camera/rgb/camera_info");

    image_sub.subscribe(nh, image_topic, 1);
    depth_sub.subscribe(nh, depth_topic, 1);

    ts_sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), image_sub, depth_sub);

    ts_sync->registerCallback(boost::bind(&rgbd_odom::imageDepthCb, this, _1, _2));

    odom_path_pub = n_p.advertise<nav_msgs::Path>("rgbd_odom/odom/path", 50);

    ROS_INFO("Waiting camera info");
    while (ros::ok())
    {
        sensor_msgs::CameraInfoConstPtr cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam_info_topic);
        if (cam_info)
        {
            cameraInfoCb(cam_info);
            break;
        }
    }

      ROS_INFO("Initializing RGBD Odometry");
        vector<int> iterCounts(4);
        iterCounts[0] = 7;
        iterCounts[1] = 7;
        iterCounts[2] = 7;
        iterCounts[3] = 10;

        vector<float> minGradMagnitudes(4);
        minGradMagnitudes[0] = 12;
        minGradMagnitudes[1] = 5;
        minGradMagnitudes[2] = 3;
        minGradMagnitudes[3] = 1;

        odom = new cv::rgbd::RgbdOdometry(
            cam_intrinsics, MIN_DEPTH, MAX_DEPTH, MAX_DEPTH_DIFF, iterCounts,
            minGradMagnitudes, MAX_POINTS_PART,
            cv::rgbd::Odometry::RIGID_BODY_MOTION);
}

void rgbd_odom::imageDepthCb(const sensor_msgs::ImageConstPtr &img_msg, const sensor_msgs::ImageConstPtr &depth_msg)
{

    ROS_INFO("Image and Depth Cb");
    cv_bridge::CvImagePtr cv_ptr;
    img_inc = true;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)

    {
        ROS_ERROR("cv_bridge RGB exception: %s", e.what());
        return;
    }

    cv_bridge::CvImagePtr cv_depth_ptr;
    try
    {
        cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception &e)

    {
        ROS_ERROR("cv_bridge DEPTH exception: %s", e.what());
        return;
    }

    if (mm_to_meters)
        cv_depth_ptr->image *= 0.001;

    if (firstImageCb)
    {
        //prevImage = cv_ptr->image;
        if (cv_ptr->image.channels() == 3)
        {
            cvtColor(cv_ptr->image, prevImage, cv::COLOR_BGR2GRAY);
        }
        else
        {
            prevImage = cv_ptr->image;
        }
        prevDepthImage = cv_depth_ptr->image;
        firstImageCb = false;
    }
    else
    {
        currImageRGB = cv_ptr->image;
        if (cv_ptr->image.channels() == 3)
        {
            cvtColor(cv_ptr->image, currImage, cv::COLOR_BGR2GRAY);
        }
        else
        {
            currImage = cv_ptr->image;
        }
        currDepthImage = cv_depth_ptr->image;
        if (!voInitialized)
            voInitialized = true;
        

    }
    frame++;
}

void rgbd_odom::cameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg)
{

    if (firstCameraInfoCb)
    {
        height = msg->height;
        width = msg->width;

        k1 = msg->D[0];
        k2 = msg->D[1];
        t1 = msg->D[2];
        t2 = msg->D[3];
        k3 = msg->D[4];

        fx = msg->K[0];
        cx = msg->K[2];
        fy = msg->K[4];
        cy = msg->K[5];


        cam_intrinsics.at<double>(0, 0) = fx;
        cam_intrinsics.at<double>(0, 2) = cx;
        cam_intrinsics.at<double>(1, 1) = fx;
        cam_intrinsics.at<double>(1, 2) = cy;
        cam_intrinsics.at<double>(2, 2) = 1;
        cout << "Cam Int" << cam_intrinsics << std::endl;
        firstCameraInfoCb = false;
    }
}

void rgbd_odom::run()
{
    if (!img_inc || !voInitialized)
        return;

    cv::Mat rigidTransform;

    bool isSuccess = odom->compute(prevImage, prevDepthImage, cv::Mat(), currImage,
                                   currDepthImage, cv::Mat(), rigidTransform);

    cv::Mat rotationMat = rigidTransform(cv::Rect(0, 0, 3, 3)).clone();
    cv::Mat translateMat = rigidTransform(cv::Rect(3, 0, 1, 3)).clone();
    // If compute successfully, then update rotationMatrix and tranlslationMatrix
    if (isSuccess == true)
    {
        if (isFirst == true)
        {
            R = rotationMat.clone();
            t = translateMat.clone();
            isFirst = false;
        }
        else
        {
             // Update Rt
             t = t + (R * translateMat);
             R = rotationMat * R;

        }
        for(unsigned int i = 0 ; i<3 ; i++)
        {
            t_f(i) = t.at<double>(i);
            for(unsigned int j = 0 ; j < 3 ; j++)
            {
                R_f(i,j) = R.at<double>(i,j);
            }
        }
       
        curr_pose.translation() = t_f;
        curr_pose.linear() = R_f;
         //Add Visual Odometry to a Path for Plotting in rviz
        addTfToPath(curr_pose);
        ROS_INFO("Visual Odometry");
        std::cout << "Translation " << t_f << std::endl;
        std::cout << "Rotation " << R_f << std::endl;

        publishOdomPath();
    }


   
    prevImage = currImage.clone();
    prevDepthImage = currDepthImage.clone();
    img_inc = false;
}

void rgbd_odom::addTfToPath(const Eigen::Affine3d &vision_pose)
{
    //Eigen::Affine3d pose=fromVisionCord(vision_pose);
    Eigen::Affine3d pose = vision_pose;
    Eigen::Quaterniond quat(pose.linear());

    geometry_msgs::PoseStamped ps;
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = "odom";
    ps.pose.position.x = pose.translation()(0);
    ps.pose.position.y = pose.translation()(1);
    ps.pose.position.z = pose.translation()(2);

    ps.pose.orientation.x = quat.x();
    ps.pose.orientation.y = quat.y();
    ps.pose.orientation.z = quat.z();
    ps.pose.orientation.w = quat.w();

    odomPath.poses.push_back(ps);
}

void rgbd_odom::publishOdomPath()
{
    nav_msgs::Path newPath = odomPath;
    newPath.header.stamp = ros::Time::now();
    newPath.header.frame_id = "odom";
    odom_path_pub.publish(newPath);
}
