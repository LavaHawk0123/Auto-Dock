#include "ArucoDetect.h"

using namespace ad;

 ArucoDetect::ArucoDetect() {

    MarkerPose.header.frame_id = "front_camera";
    ROS_INFO_STREAM("Started Aruco Detection Node");
    pub_pose = nh.advertise < geometry_msgs::PoseStamped > ("/marker/globalpose",100);
    pub_marker = nh.advertise < sensor_msgs::Image > ("/arucomarker/image_raw", 100);					// Publisher for Image of detection
    pub_arucoMarkerInfo = nh.advertise< arucoDetection::ArucoMsg > ("/arucomarker/info",100);
    pub_velocity = nh.advertise < geometry_msgs::Twist > ("/cmd_vel", 1);							// Publisher for rover velocity
    viz_marker_pub = nh.advertise < visualization_msgs::Marker >("/tag_vis_arucomarker",100);					// Publisher for Visual Marker of Tag
    sub_camera_sim = nh.subscribe("/camera/rgb/image_rect_color", 100, &  ArucoDetect::imageCallback, this);			// Callback for Image from simulation
    tf_sub_sim = nh.subscribe("/front_camera/color/image_raw", 100, &ArucoDetect::getTransforms, this);
    //sub_camera = nh.subscribe("/camera/color/image_raw", 100, & ArucoDetect::imageCallback,this);		//  Callback for Image from camera - Realsense d435
    //tf_sub = nh.subscribe("/camera/color/image_raw", 100, &ArucoDetect::getTransforms, this);
    odom_sub = nh.subscribe("/odom", 100, &ArucoDetect::poseCallback, this);
}

void  ArucoDetect::imageCallback(const sensor_msgs::ImageConstPtr & msg) {

    camera_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    arucoDetection::ArucoMsg arucoInfoMsg;
    ArucoDetect::detect_aruco();
    estimate_pose();
    msg_pub = camera_image -> toImageMsg();
    pub_marker.publish(msg_pub);
    
};

void ArucoDetect::getTransforms(const sensor_msgs::ImageConstPtr & msg){
      tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener tfListener(tfBuffer);
      geometry_msgs::TransformStamped odom_to_base,base_to_cam;
      while(odom_to_base.transform.translation.x==0 && base_to_cam.transform.translation.x==0 && ros::ok()){
        try{
                odom_to_base = tfBuffer.lookupTransform("base_link", "odom",ros::Time(0));
                base_to_cam = tfBuffer.lookupTransform("front_camera", "base_link",ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
        }
    }

    odom_to_base_tf = odom_to_base;
    base_to_cam_tf = base_to_cam;
    //ROS_INFO_STREAM("odom_to_base Transform : \n"<<odom_to_base_tf.transform.translation<<"\n");
    //ROS_INFO_STREAM("base_to_cam Transform : \n"<<base_to_cam_tf.transform.rotation<<"\n");
}
void ArucoDetect::poseCallback(const nav_msgs::Odometry msg){
    //geometry_msgs::Pose cur_pose = msg.pose.pose;
    if (marker_ids_detected.size() > 0) {
        geometry_msgs::PoseStamped cur_pose;
        cur_pose.pose =  arucoInfoMsg.odom.pose.pose; 
        tf2::doTransform(cur_pose, cur_pose, base_to_cam_tf); 
        //tf2::doTransform(cur_pose, cur_pose, odom_to_base_tf); 
        cur_pose.header.seq = 1;
        cur_pose.header.stamp = ros::Time::now();
        cur_pose.header.frame_id = "base_link";
        cur_pose.pose.position.y = -cur_pose.pose.position.y;
        pub_pose.publish(cur_pose);
        ROS_INFO_STREAM("Pose in Global Frame : "<<cur_pose<<"\n");
    }

};


void ArucoDetect::getGlobalPose(){
    //tf2::doTransform(robot_pose, robot_pose, base_to_cam_tf); 
};


void  ArucoDetect::detect_aruco() {
    //set_params();
    markers_drawn_img = camera_image -> image;
    cv::aruco::detectMarkers(markers_drawn_img, dictionary, corners, marker_ids_detected, params);
    marker_ID.data = " ";
    if (marker_ids_detected.size() > 0) {
        cv::aruco::drawDetectedMarkers(markers_drawn_img, corners, marker_ids_detected);
    }
};

/*void  ArucoDetect::set_params() {

    params -> cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    params -> adaptiveThreshWinSizeMin = 5;
    params -> adaptiveThreshWinSizeMax = 20;
    params -> adaptiveThreshWinSizeStep = 5;
    params -> adaptiveThreshConstant = 25;
    params -> minMarkerPerimeterRate = 0.03;
    params -> maxMarkerPerimeterRate = 1;
    params -> polygonalApproxAccuracyRate = 0.052;
    params -> minCornerDistanceRate = 0.05;
    params -> minMarkerDistanceRate = 0.1;
    params -> minDistanceToBorder = 2;
    params -> cornerRefinementWinSize = 4;
    params -> cornerRefinementMinAccuracy = 0.1;
    params -> cornerRefinementMaxIterations = 50;
    params->minOtsuStdDev = 0.1;

};*/

void  ArucoDetect::estimate_pose() {
    final_image = camera_image -> image;
    // Camera Caliberation for the Intel Realsense D435i
    cv::Matx33d cameraMatrix{632.29863082751251, 0, 319.5, 0, 632.29863082751251, 239.5, 0, 0, 1};
    cv::Mat distortionCoeffs = (cv::Mat_<double>(5,1) << 0.070528331223347215, 0.26247385180956367, 0, 0, -1.0640942232949715);
    // Enter and calculate pose only if a marker is detected
    if (marker_ids_detected.size() > 0) {
        cv::aruco::estimatePoseSingleMarkers(corners, 0.02, cameraMatrix, distortionCoeffs, RotationalVectors, TranslationalVectors);
        try {
            cv::aruco::drawAxis(final_image, cameraMatrix, distortionCoeffs, RotationalVectors[0], TranslationalVectors[0], 0.05);
        } 
        catch (...) {
            ROS_INFO_STREAM("\n Unable to draw axis.");
        }
        // Use the marker corners to calculate position  
        calculate_pose();
    }
};


void ArucoDetect::calculate_pose() {
    // Choose the first market detected
        curr_RotVec = RotationalVectors[0];
        curr_TransVec = TranslationalVectors[0];
    // Find the rotational matrix from the Rotational Vectors
        cv::Rodrigues(curr_RotVec, rotMat);
        //ROS_INFO_STREAM(rotMat);
    // if the Rotational Matrix found is valid, proceed
        if (cv::determinant(rotMat) > 0.99 && cv::determinant(rotMat) < 1.01) {
            // Setting values for pose of Marker
            arucoInfoMsg.rotational_vector.at(0) = curr_RotVec[0];
            arucoInfoMsg.rotational_vector.at(1) = curr_RotVec[1];
            arucoInfoMsg.rotational_vector.at(2) = curr_RotVec[2];
            arucoInfoMsg.translational_vector.at(0) = curr_TransVec[0];
            arucoInfoMsg.translational_vector.at(1) = curr_TransVec[1];
            arucoInfoMsg.translational_vector.at(2) = curr_TransVec[2];
            // Transforming origin of the camera
            origin = rotMat * -curr_TransVec;
            // finding angle between bearing and centre of Aruco Tag
            aruco_yaw = cvFastArctan(curr_TransVec[0],curr_TransVec[2]);
            //std::cout << "aruco yaw: " << aruco_yaw << std::endl;
            //curr_TransVec[1] = -curr_TransVec[1];
            curr_TransVec[0] = curr_TransVec[0];             
            tf2::Quaternion q,v;
            q[0] = -std::abs(std::sqrt(std::pow(curr_RotVec[0],2)+std::pow(curr_RotVec[1],2)+std::pow(curr_RotVec[2],2)));
            q[1] = curr_RotVec[0];
            q[2] = curr_RotVec[1];
            q[3] = curr_RotVec[2];
            v.setRPY( 0,0,1.57);
            v.normalize();
            q = v*q;
            q = q.normalize();
            quaternion = tf2::toMsg(q);
            markerDistance = sqrt((curr_TransVec[1] * curr_TransVec[1]) + (curr_TransVec[2] * curr_TransVec[2]))*5;

        // Publish a marker to depict tags
        arucoInfoMsg.odom.pose.pose.position.x = curr_TransVec[2]+1;
        arucoInfoMsg.odom.pose.pose.position.y = curr_TransVec[0];
        arucoInfoMsg.odom.pose.pose.position.z = curr_TransVec[1];
        arucoInfoMsg.odom.pose.pose.orientation = quaternion;
            
        //ROS_INFO_STREAM("Aruco Marker Pose : \n");
        //ROS_INFO_STREAM("x : " << curr_TransVec[2]);
        //ROS_INFO_STREAM("y : " << curr_TransVec[0]);
        //ROS_INFO_STREAM("z : " << curr_TransVec[1]);
        //ROS_INFO_STREAM("Marker Distance : " << markerDistance);

        publishArucoInfo();
        pub_arucoMarkerInfo.publish(arucoInfoMsg);
        publishMarker(curr_RotVec,curr_TransVec,quaternion);
    }
};

void ArucoDetect::publishArucoInfo(){
        arucoInfoMsg.distance = markerDistance;
        arucoInfoMsg.id = marker_ids_detected[0];
        if(aruco_yaw>180){
            aruco_yaw = aruco_yaw-360;
        }
        arucoInfoMsg.yaw = aruco_yaw;
        try {
            pub_arucoMarkerInfo.publish(arucoInfoMsg);
        }
        catch(...){
            ROS_INFO_STREAM("Couldn't publish marker info \n");
        }
};

void  ArucoDetect::publishMarker(cv::Vec3d curr_RotVec,cv::Vec3d curr_TransVec,geometry_msgs::Quaternion quaternion){
            visualization_msgs::Marker marker;
		    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
		    marker.header.frame_id = "front_camera";
		    marker.header.stamp = ros::Time::now();

		    // Set the namespace and id for this marker.  This serves to create a unique ID
		    // Any marker sent with the same namespace and id will overwrite the old one
		    marker.ns = "basic_shapes";
		    marker.id = 0;

		    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		    marker.type = shape;

		    // Set the marker action.  Options are ADD, DELETE
		    marker.action = visualization_msgs::Marker::ADD;

		    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		    marker.pose.position.x = curr_TransVec[2];
		    marker.pose.position.y = curr_TransVec[0];
		    marker.pose.position.z = curr_TransVec[1];
		    marker.pose.orientation = quaternion;

		    // Set the scale of the marker -- 1x1x1 here means 1m on a side
		    marker.scale.x = 1.0;
		    marker.scale.y = 0.1;
		    marker.scale.z = 1.0;

		    // Set the color 
		    marker.color.r = 0.0f;
		    marker.color.g = 1.0f;
		    marker.color.b = 0.0f;
		    marker.color.a = 1.0;
		    
		    marker.text = "Distance = "+std::to_string(markerDistance);
			
		    // Publish the marker
		    viz_marker_pub.publish(marker);
};


int main(int argc, char ** argv) {

    ros::init(argc, argv, "aruco_detection_node");
    ros::AsyncSpinner spinner(5);
    spinner.start();
    ArucoDetect object;
    ros::waitForShutdown();
}
