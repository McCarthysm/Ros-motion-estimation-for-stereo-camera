#include "motionEstimation_node.h"


motionEstimation_node::motionEstimation_node()
{
    ros::NodeHandle nh ("~");
    int mode = 0;
    std::string datapath;
    cv::FileStorage config("data/config.yml", cv::FileStorage::READ);
    config["mode"] >> mode;
    config["path"] >> datapath;
    config.release();
    cv::FileStorage fs(dataPath + "disparity/disparity_0.yml", cv::FileStorage::READ);
    fs["Q"] >> Q;
    fs.release();
    Camera camera;
    std::vector<Camera> cam;
    image_sub_count = 0;
    loadIntrinsic(datapath, K_L, K_R, distCoeff_L, distCoeff_R);
     loadExtrinsic(datapath, R_LR, T_LR, E_LR, F_LR);
     K_L.convertTo(K_L, CV_32F);
     K_R.convertTo(K_R, CV_32F);
     distCoeff_L.convertTo(distCoeff_L, CV_32F);
     distCoeff_R.convertTo(distCoeff_R, CV_32F);
     E_LR.convertTo(E_LR, CV_32F);
     F_LR.convertTo(F_LR, CV_32F);
     R_LR.convertTo(R_LR, CV_32F);
     T_LR.convertTo(T_LR, CV_32F);
     Q.convertTo(Q, CV_32F);
     cv::invert(K_L, KInv_L);
     cv::invert(K_R, KInv_R);
    // P_LR, rvec_LR;
     composeProjectionMat(T_LR, R_LR, P_LR);
     cv::Rodrigues(R_LR, rvec_LR);
     P_0 = (cv::Mat_<float>(3,4) <<
                        1.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, 0.0 );
    decomposeProjectionMat(P_0, R_0, T_0);
    currentPos_ES_L = cv::Mat::eye(4, 4, CV_32F);
    currentPos_ES_R = cv::Mat::eye(4, 4, CV_32F);
    currentPos_ES_mean = cv::Mat::eye(4, 4, CV_32F);
    currentPos_PnP_L = cv::Mat::eye(4, 4, CV_32F);
    currentPos_PnP_R = cv::Mat::eye(4, 4, CV_32F);
    currentPos_Stereo = cv::Mat::eye(4, 4, CV_32F);
    features = getStrongFeaturePoints(left, 100, 0.001, 20);
   // std::vector<cv::Point2f> points_L1_temp, points_R1_temp;

    //getFiles(datapath + "left/", filenames_left);
    //getFiles(datapath + "right/", filenames_right);
     image_transport::ImageTransport it(nh);
    nh.param("Camera_left_topic",c_left, std::string("left"));
    nh.param("Camera_right_topic",c_right,std::string("right"));
    nh.param("base_frame", p_base_frame_, std::string("base_link"));
    nh.param("map_frame", p_map_frame_, std::string("map"));
    nh.param("odom_frame", p_odom_frame_, std::string("odom"));
     cv::namedWindow("left");
     cv::namedWindow("right");
     skipFrameNumber = 0;
    
     image_sub = it.subscribe(c_left, 50, &motionEstimation_node::imageCallback, this);
     image_r = it.subscribe(c_right, 50, &motionEstimation_node::imageCallback, this);

     posePublisher_ = node_.advertise<geometry_msgs::Pose>("slam_out_pose", 1, false);
}

cv::Mat motionEstimation_node::imageCallback(sensor_msgs::ImagePtr &left, sensor_msgs::ImagePtr &right)
{
   /* try
    {
        cv::imshow("left", cv_bridge::toCvShare(left, "bgr8")->image);
        cv::imshow("right", cv_bridge::toCvShare(right, "bgr8")->image);
        cv::waitKey(30);
    }*/
      if(image_sub_count == 0)
        {

        //right;
          features = getStrongFeaturePoints(image_L1, 100, 0.001, 20);

          refindFeaturePoints(left, right, features, points_L1_temp, points_R1_temp);
           if (10 > points_L1_temp.size())
           {
               ROS_INFO("Could not find more than features in stereo 1:");
           }
           else
           {
               camera.left = left;
               camera.right = right;
               cam.push_back(camera);

           }

        }

       image_sub_count++;

      while(image_sub_count > 0)
      {
              skipFrameNumber++;
          if(4 < skipFrameNumber)
          {
             ROS_INFO("NO MOVEMENT FOR LAST 4 FRAMES");
             image_sub_count = 0;
             break;
          }
          refindFeaturePoints(camera.left, left, points_L1_temp, points_L1, points_L2);
          refindFeaturePoints(camera.right, right, points_R1_temp, points_R1, points_R2);
          deleteUnvisiblePoints(points_L1_temp, points_R1_temp, points_L1, points_R1, points_L2, points_R2, camera.left.cols, camera.left.rows);

          if(0 == points_L1.size())
          {
              image_sub_count--;
              ROS_INFO("Could not find more than features in stereo 2:");
              break;
          }

          if (1 == mode)
          {
             if (8 > points_L1.size())
             {
                ROS_INFO("NO MOVEMENT: to less points found");
                image_sub_count--;
                break;
             }

             getInliersFromHorizontalDirection(make_pair(points_L1, points_R1), inliersHorizontal_L1, inliersHorizontal_R1);
             getInliersFromHorizontalDirection(make_pair(points_L2, points_R2), inliersHorizontal_L2, inliersHorizontal_R2);
             deleteZeroLines(points_L1, points_R1, points_L2, points_R2, inliersHorizontal_L1, inliersHorizontal_R1, inliersHorizontal_L2, inliersHorizontal_R2);

             if (8 > inliersHorizontal_L1.size())
             {
                ROS_INFO("NO MOVEMENT: couldn't find horizontal points... probably rectification fails or to less feature points found?!");
                image_sub_count--;
                break;
             }

            foundF_L = getFundamentalMatrix(points_L1, points_L2, &inliersF_L1, &inliersF_L2, F_L);
            foundF_R = getFundamentalMatrix(points_R1, points_R2, &inliersF_R1, &inliersF_R2, F_R);
            deleteZeroLines(inliersF_L1, inliersF_L2, inliersF_R1, inliersF_R2);

            if (1 > inliersF_L1.size())
            {
                ROS_INFO("NO MOVEMENT: couldn't find enough ransac inlier");
                image_sub_count--;
                break;
            }

            drawCorresPoints(camera.left, inliersF_L1, inliersF_L2, "inlier F left " , CV_RGB(0,0,255));
            drawCorresPoints(camera.right, inliersF_R1, inliersF_R2, "inlier F right " , CV_RGB(0,0,255));
            bool poseEstimationFoundES_L = false;
            bool poseEstimationFoundES_R = false;

            if(foundF_L)
            {
              poseEstimationFoundES_L = motionEstimationEssentialMat(inliersF_L1, inliersF_L2, F_L, K_L, T_E_L, R_E_L);
            }

            if(foundF_R)
            {
              poseEstimationFoundES_R = motionEstimationEssentialMat(inliersF_R1, inliersF_R2, F_R, K_R, T_E_R, R_E_R);
            }

            if (!poseEstimationFoundES_L && !poseEstimationFoundES_R)
            {
               image_sub_count--;
               break;
               T_E_L = cv::Mat::zeros(3, 1, CV_32F);
               R_E_L = cv::Mat::eye(3, 3, CV_32F);
               T_E_R = cv::Mat::zeros(3, 1, CV_32F);
               R_E_R = cv::Mat::eye(3, 3, CV_32F);
            }

            else if (!poseEstimationFoundES_L)
            {
               T_E_L = cv::Mat::zeros(3, 1, CV_32F);
               R_E_L = cv::Mat::eye(3, 3, CV_32F);
            }

            else if (!poseEstimationFoundES_R)
            {
               T_E_R = cv::Mat::zeros(3, 1, CV_32F);
               R_E_R = cv::Mat::eye(3, 3, CV_32F);
            }

            cv::Mat PK_0 = K_L * P_0;
            cv::Mat PK_LR = K_R * P_LR;

            TriangulatePointsHZ(PK_0, PK_LR, points_L1, points_R1, 0, pointCloud_1);
            TriangulatePointsHZ(PK_0, PK_LR, points_L2, points_R2, 0, pointCloud_2);
#if 1
            composeProjectionMat(T_E_L, R_E_L, P_L);
            composeProjectionMat(T_E_R, R_E_R, P_R);
            cv::Mat PK_L = K_L * P_L;
            cv::Mat PK_R = K_R * P_R;
            getScaleFactor(PK_0, PK_LR, PK_L, PK_R, points_L1, points_R1, points_L2, points_R2, u_L1, u_R1, stereoCloud, nearestPoints);
            ROS_INFO( "skipFrameNumber :  %d",  skipFrameNumber);
            if(u_L1 < -1 || u_L1 > 1000*skipFrameNumber)
            {
               ROS_INFO("scale factors for left cam is too big: %d ", u_L1);
               //skipFrame = true;
               //continue;
            }
            else
            {
                T_E_L = T_E_L * u_L1;
            }

            if(u_R1 < -1 || u_R1 > 1000*skipFrameNumber )
            {
                ROS_INFO( "scale factors for right cam is too big: %d ", u_R1);
                                //skipFrame = true;
                                //continue;
            }
            else
            {
                T_E_R = T_E_R * u_R1;
            }

#if 0
                // get RGB values for pointcloud representation
                std::vector<cv::Vec3b> RGBValues;
                for (unsigned int i = 0; i < points_L1.size(); ++i){
                    uchar grey = camera.left.at<uchar>(points_L1[i].x, points_L1[i].y);
                    RGBValues.push_back(cv::Vec3b(grey,grey,grey));
                }

                std::vector<cv::Vec3b> red;
                for (unsigned int i = 0; i < 5; ++i){
                    red.push_back(cv::Vec3b(0,0,255));
                }

                AddPointcloudToVisualizer(stereoCloud, "cloud1" + std::to_string(frame1), RGBValues);
                AddPointcloudToVisualizer(nearestPoints, "cloud2" + std::to_string(frame1), red);
#endif
#else
                // 2. method:
                float u_L2, u_R2;
                getScaleFactor2(T_LR, R_LR, T_E_L, R_E_L, T_E_R, u_L2, u_R2);

                if(u_L2 < -1000 || u_R2 < -1000 || u_L2 > 1000 || u_R2 > 1000 ){
                    std::cout << "scale factors to small or to big:  L: " << u_L2 << "  R: " << u_R2  << std::endl;
                } else {
                    T_E_L = T_E_L * u_L2;
                    T_E_R = T_E_R * u_R2;
                }

                //compare both methods
                ROS_INFO("u links  2:%d ", u_L2);
                ROS_INFO("u rechts 2:%d", u_R2);
#endif
                ROS_INFO("translation 1:%d ", T_E_L);
                cv::Mat newTrans3D_E_L;
                getNewTrans3D( T_E_L, R_E_L, newTrans3D_E_L);


                cv::Mat newPos_ES_L;
                getAbsPos(currentPos_ES_L, newTrans3D_E_L, R_E_L.t(), newPos_ES_L);
                cv::Mat rotation_ES_L, translation_ES_L;
                decomposeProjectionMat(newPos_ES_L, translation_ES_L, rotation_ES_L);
                cv::Mat newPos_ES_mean = newPos_ES_L + newPos_ES_R;
                newPos_ES_mean /= 2;

                cv::Mat rotation_ES_mean, translation_ES_mean;
                decomposeProjectionMat(newPos_ES_mean, translation_ES_mean, rotation_ES_mean);
                geometry_msgs::Pose pose;
                pose.position.x = rotation_ES_mean(0);
                pose.position.y = rotation_ES_mean(1);
                pose.position.z = rotation_ES_mean(2);
                 posePublisher_.publish(pose);
                Eigen::Matrix3f m;
                m = translation_ES_mean;
                float roll;
                float pitch;
                float yaw;
                roll = atan2(m(3,2), m(3,3));
                pitch = atan(-m(3,1)/(m(3,2)*sin(roll)+m(3,3)*cos(roll)));
                yaw = atan2(m(2,1),m(1,1));

                currentPos_ES_mean = newPos_ES_mean;
                currentPos_ES_L = newPos_ES_L;
                currentPos_ES_R = newPos_ES_R;

                ROS_INFO("abs. position  ", translation_ES_mean);


                

          }

          if(2 == mode)
          {
              if (8 > points_L1.size())
              {
                  ROS_INFO("NO MOVEMENT: to less points found");
                  image_sub_count--;
                  break;
              }

              std::vector<cv::Point2f> inliersHorizontal_L1, inliersHorizontal_R1, inliersHorizontal_L2, inliersHorizontal_R2;
              getInliersFromHorizontalDirection(make_pair(points_L1, points_R1), inliersHorizontal_L1, inliersHorizontal_R1);
              getInliersFromHorizontalDirection(make_pair(points_L2, points_R2), inliersHorizontal_L2, inliersHorizontal_R2);
              //delete all points that are not correctly found in stereo setup
              deleteZeroLines(points_L1, points_R1, points_L2, points_R2, inliersHorizontal_L1, inliersHorizontal_R1, inliersHorizontal_L2, inliersHorizontal_R2);

              if (8 > points_L1.size())
              {
                   ROS_INFO("NO MOVEMENT: couldn't find horizontal points... probably rectification fails or to less feature points found?!");
                   image_sub_count--;
                   break;
              }

              cv::Mat F_L;
              bool foundF_L;
              std::vector<cv::Point2f> inliersF_L1, inliersF_L2;
              foundF_L = getFundamentalMatrix(points_L1, points_L2, &inliersF_L1, &inliersF_L2, F_L);

              // compute fundemental matrix F_R1R2 and get inliers from Ransac
              cv::Mat F_R;
              bool foundF_R;
              std::vector<cv::Point2f> inliersF_R1, inliersF_R2;
              foundF_R = getFundamentalMatrix(points_R1, points_R2, &inliersF_R1, &inliersF_R2, F_R);

              // make sure that there are all inliers in all frames.
              deleteZeroLines(inliersF_L1, inliersF_L2, inliersF_R1, inliersF_R2);

              drawCorresPoints(image_R1, inliersF_R1, inliersF_R2, "inlier F right " , CV_RGB(0,0,255));
              drawCorresPoints(image_L1, inliersF_L1, inliersF_L2, "inlier F left " , CV_RGB(0,0,255));

              // calibrate projection mat
              cv::Mat PK_0 = K_L * P_0;
              cv::Mat PK_LR = K_R * P_LR;

              std::vector<cv::Point3f> pointCloud_1, pointCloud_2;
              TriangulatePointsHZ(PK_0, PK_LR, inliersF_L1, inliersF_R1, 0, pointCloud_1);
              TriangulatePointsHZ(PK_0, PK_LR, inliersF_L2, inliersF_R2, 0, pointCloud_2);

#if 1
                //LEFT:
                bool poseEstimationFoundTemp_L = false;
                cv::Mat T_PnP_L, R_PnP_L;
                if(foundF_L){
                    // GUESS TRANSLATION + ROTATION UP TO SCALE!!!
                    poseEstimationFoundTemp_L = motionEstimationEssentialMat(inliersF_L1, inliersF_L2, F_L, K_L, T_PnP_L, R_PnP_L);
                }

                if (!poseEstimationFoundTemp_L){
                    image_sub_count--;
                    break;
                }

#if 0
                // scale factor:
                float u_L1;
                cv::Mat P_L;
                composeProjectionMat(T_PnP_L, R_PnP_L, P_L);

                // calibrate projection mat
                cv::Mat PK_L = K_L * P_L;

                getScaleFactorLeft(PK_0, PK_LR, PK_L, inliersF_L1, inliersF_R1, inliersF_L2, u_L1);
                if(u_L1 < -1 || u_L1 > 1000 ){
                    ROS_INFO("scale factors to small or to big:  L: ", u_L1);
                    image_sub_count--;
                    break;
                }

                T_PnP_L = T_PnP_L * u_L1;
#endif

                bool poseEstimationFoundPnP_L = motionEstimationPnP(inliersF_L2, pointCloud_1, K_L, T_PnP_L, R_PnP_L);

                if (!poseEstimationFoundPnP_L)
                {
                      image_sub_count--;
                      break;
                }

                if(cv::norm(T_PnP_L) > 1500.0 * skipFrameNumber)
                {
                  // this is bad...
                  ROS_INFO("NO MOVEMENT: estimated camera movement is too big, skip this camera.. T = ", cv::norm(T_PnP_L));
                  image_sub_count--;
                  break;
                }

                cv::Mat newTrans3D_PnP_L;
                getNewTrans3D( T_PnP_L, R_PnP_L, newTrans3D_PnP_L);

                cv::Mat newPos_PnP_L;
                getAbsPos(currentPos_PnP_L, newTrans3D_PnP_L, R_PnP_L, newPos_PnP_L);

                cv::Mat rotation_PnP_L, translation_PnP_L;
                decomposeProjectionMat(newPos_PnP_L, translation_PnP_L, rotation_PnP_L);

                pose.position.x = rotation_PnP_L(0);
                pose.position.y = rotation_PnP_L(1);
                pose.position.z = rotation_PnP_L(2);
                posePublisher_.publish(pose);
                Eigen::Matrix3f m;
                m = translation_ES_mean;
                float roll;
                float pitch;
                float yaw;
                roll = atan2(m(3,2), m(3,3));
                pitch = atan(-m(3,1)/(m(3,2)*sin(roll)+m(3,3)*cos(roll)));
                yaw = atan2(m(2,1),m(1,1));
                currentPos_PnP_L  = newPos_PnP_L ;
#else
              //RIGHT:
                             bool poseEstimationFoundTemp_R = false;
                             cv::Mat  T_PnP_R, R_PnP_R;
                             if(foundF_R){
                                 // GUESS TRANSLATION + ROTATION UP TO SCALE!!!
                                 poseEstimationFoundTemp_R = motionEstimationEssentialMat(inliersF_R1, inliersF_R2, F_R, K_R, KInv_R, T_PnP_R, R_PnP_R);
                             }

                             if (!poseEstimationFoundTemp_R){
                                 skipFrame = true;
                                 continue;
                             }

                             // use initial guess values for pose estimation
                             bool poseEstimationFoundPnP_R = motionEstimationPnP(inliersF_R2, pointCloud_1, K_R, T_PnP_R, R_PnP_R);

                             if (!poseEstimationFoundPnP_R){
                                 skipFrame = true;
                                 continue;
                             }

                             cv::Mat newTrans3D_PnP_R;
                             getNewTrans3D( T_PnP_R, R_PnP_R, newTrans3D_PnP_R);

                             cv::Mat newPos_PnP_R;
                             getAbsPos(currentPos_PnP_R, newTrans3D_PnP_R, R_PnP_R, newPos_PnP_R);

                             cv::Mat rotation_PnP_R, translation_PnP_R;
                             decomposeProjectionMat(newPos_PnP_R, translation_PnP_R, rotation_PnP_R);
                             pose.position.x = rotation_PnP_R(0);
                             pose.position.y = rotation_PnP_R(1);
                             pose.position.z = rotation_PnP_R(2);
                             posePublisher_.publish(pose);
                             Eigen::Matrix3f m;
                             m = translation_ES_mean;
                             float roll;
                             float pitch;
                             float yaw;
                             roll = atan2(m(3,2), m(3,3));
                             pitch = atan(-m(3,1)/(m(3,2)*sin(roll)+m(3,3)*cos(roll)));
                             yaw = atan2(m(2,1),m(1,1));
                             currentPos_PnP_R  = newPos_PnP_R ;
#endif
          }

                             if (3 == mode)
                             {
                                 std::vector<cv::Point2f> inliersHorizontal_L1, inliersHorizontal_R1, inliersHorizontal_L2, inliersHorizontal_R2;
                                 getInliersFromHorizontalDirection(make_pair(points_L1, points_R1), inliersHorizontal_L1, inliersHorizontal_R1);
                                 getInliersFromHorizontalDirection(make_pair(points_L2, points_R2), inliersHorizontal_L2, inliersHorizontal_R2);
                                 //delete all points that are not correctly found in stereo setup
                                 deleteZeroLines(points_L1, points_R1, points_L2, points_R2, inliersHorizontal_L1, inliersHorizontal_R1, inliersHorizontal_L2, inliersHorizontal_R2);

                                 if (8 > points_L1.size())
                                 {
                                     ROS_INFO("NO MOVEMENT: couldn't find horizontal points... probably rectification fails or to less feature points found?!");
                                     image_sub_count--;
                                     break;
                                 }

                                 drawCorresPoints(image_L1, points_L1, points_R1, "inlier F1 links rechts", cv::Scalar(255,255,0));
                                 drawCorresPoints(image_L2, points_L2, points_R2, "inlier F2 links rechts", cv::Scalar(255,255,0));

                                                // calibrate projection mat
                                 cv::Mat PK_0 = K_L * P_0;
                                 cv::Mat PK_LR = K_R * P_LR;

                                                // TRIANGULATE POINTS
                                 std::vector<cv::Point3f> pointCloud_1, pointCloud_2;
                                 TriangulatePointsHZ(PK_0, PK_LR, points_L1, points_R1, 0, pointCloud_1);
                                 TriangulatePointsHZ(PK_0, PK_LR, points_L2, points_R2, 0, pointCloud_2);


                                 float reproj_error_1L = calculateReprojectionErrorHZ(PK_0, points_L1, pointCloud_1);
                                 float reproj_error_1R = calculateReprojectionErrorHZ(PK_LR, points_R1, pointCloud_1);

                                 if (!positionCheck(P_0, pointCloud_2) && !positionCheck(P_LR, pointCloud_2) && reproj_error_2L < 10.0 && reproj_error_2R < 10.0 )
                                 {
                                        ROS_INFO("second pointcloud seem's to be not perfect..");
                                        image_sub_count--;
                                        break;
                                 }

#if 0
                //load disparity map
                cv::Mat dispMap1;
                cv::FileStorage fs_dist1(dataPath + "disparity/disparity_"+to_string(frame)+".yml", cv::FileStorage::READ);
                fs_dist1["disparity"] >> dispMap1;
                fs_dist1.release();

                cv::Mat dispMap2;
                cv::FileStorage fs_dist2(dataPath + "disparity/disparity_"+to_string(frame+1)+".yml", cv::FileStorage::READ);
                fs_dist2["disparity"] >> dispMap2;
                fs_dist2.release();

                dispMap1.convertTo(dispMap1, CV_32F);
                dispMap2.convertTo(dispMap2, CV_32F);

                std::vector <cv::Mat_<float>> cloud1;
                std::vector <cv::Mat_<float>> cloud2;
                for(unsigned int i = 0; i < inlier_median_L1.size(); ++i){
                    cv::Mat_<float> point3D1(1,4);
                    cv::Mat_<float> point3D2(1,4);
                    calcCoordinate(point3D1, Q, dispMap1, inlier_median_L1[i].x, inlier_median_L1[i].y);
                    calcCoordinate(point3D2, Q, dispMap2, inlier_median_L2[i].x, inlier_median_L2[i].y);
                    cloud1.push_back(point3D1);
                    cloud2.push_back(point3D2);
                }

                std::vector<cv::Point3f> pcloud1, pcloud2;
                std::vector<cv::Vec3b> rgb1, rgb2;
                for (unsigned int i = 0; i < cloud1.size(); ++i) {
                    if (!cloud1[i].empty() && !cloud2[i].empty()){
                        pcloud1.push_back(cv::Point3f(cloud1[i](0), cloud1[i](1), cloud1[i](2) ));
                                          pcloud2.push_back(cv::Point3f(cloud2[i](0), cloud2[i](1), cloud2[i](2) ));
                                                            rgb1.push_back(cv::Vec3b(255,0,0));
                                          rgb2.push_back(cv::Vec3b(0,255,0));
                    }
                }

                AddPointcloudToVisualizer(pcloud1, "pcloud1", rgb1);
                AddPointcloudToVisualizer(pcloud2, "pcloud2", rgb2);

                cv::Mat T_Stereo, R_Stereo;
                bool poseEstimationFoundStereo = motionEstimationStereoCloudMatching(pcloud1, pcloud2, T_Stereo, R_Stereo);

#else

                cv::Mat T_Stereo, R_Stereo;
                bool poseEstimationFoundStereo = motionEstimationStereoCloudMatching(pointCloud_1, pointCloud_2, T_Stereo, R_Stereo);
#endif

                if (!poseEstimationFoundStereo)
                {
                      image_sub_count--;
                      break;
                }
                ROS_INFO("ROTATION \n");
                ROS_INFO(R_Stereo);
                ROS_INFO("\n TRANSLATION \n");
                ROS_INFO(T_Stereo);

                float x_angle, y_angle, z_angle;
                decomposeRotMat(R_Stereo, x_angle, y_angle, z_angle);
                ROS_INFO("x angle:", x_angle);
                ROS_INFO("y angle:", y_angle);
                ROS_INFO("z angle:", z_angle);

                cv::Mat newTrans3D_Stereo;
                getNewTrans3D( T_Stereo, R_Stereo, newTrans3D_Stereo);

                cv::Mat newPos_Stereo;
                getAbsPos(currentPos_Stereo, newTrans3D_Stereo, R_Stereo, newPos_Stereo);

                cv::Mat rotation, translation;
                decomposeProjectionMat(newPos_Stereo, translation, rotation);

                pose.position.x = rotation(0);
                pose.position.y = rotation(1);
                pose.position.z = rotation(2);
                posePublisher_.publish(pose);
                Eigen::Matrix3f m;
                m = translation;
                float roll;
                float pitch;
                float yaw;
                roll = atan2(m(3,2), m(3,3));
                pitch = atan(-m(3,1)/(m(3,2)*sin(roll)+m(3,3)*cos(roll)));
                yaw = atan2(m(2,1),m(1,1));

                currentPos_Stereo = newPos_Stereo;



                             }

                             if (4 == mode)

                             {
                                             // ######################## TRIANGULATION TEST ################################
                                             // get inlier from stereo constraints
                                             std::vector<cv::Point2f> inliersHorizontal_L1, inliersHorizontal_R1, inliersHorizontal_L2, inliersHorizontal_R2;
                                             getInliersFromHorizontalDirection(make_pair(points_L1, points_R1), inliersHorizontal_L1, inliersHorizontal_R1);
                                             getInliersFromHorizontalDirection(make_pair(points_L2, points_R2), inliersHorizontal_L2, inliersHorizontal_R2);
                                             //delete all points that are not correctly found in stereo setup
                                             deleteZeroLines(points_L1, points_R1, points_L2, points_R2, inliersHorizontal_L1, inliersHorizontal_R1, inliersHorizontal_L2, inliersHorizontal_R2);

                                             drawCorresPoints(image_L1, points_L1, points_R1, "inlier 1 " , CV_RGB(0,0,255));
                                             drawCorresPoints(image_R1, points_L2, points_R2, "inlier 2 " , CV_RGB(0,0,255));

                                             if(0 == points_L1.size()){
                                                 image_sub_count--;
                                                 break;
                                             }

                                             // calibrate projection mat
                                             cv::Mat PK_0 = K_L * P_0;
                                             cv::Mat PK_LR = K_R * P_LR;

                                             std::vector<cv::Point3f> pointCloud_1, pointCloud_2;
                                             TriangulatePointsHZ(PK_0, PK_LR, points_L1, points_R1, 0, pointCloud_1);
                                             TriangulatePointsHZ(PK_0, PK_LR, points_L2, points_R2, 0, pointCloud_2);



                                             if(0 == pointCloud_1.size())
                                             {
                                                  ROS_INFO("horizontal inlier: can't find any corresponding points in all 4 frames' ");
                                                  image_sub_count--;
                                                  break;
                                             }


                                                             // get RGB values for pointcloud representation
                                              std::vector<cv::Vec3b> RGBValues;
                                              for (unsigned int i = 0; i < points_L1.size(); ++i)
                                              {
                                                   uchar grey = image_L1.at<uchar>(points_L1[i].x, points_L1[i].y);
                                                   RGBValues.push_back(cv::Vec3b(grey,grey,grey));
                                              }

                                                   AddPointcloudToVisualizer(pointCloud_1, "cloud1" + std::to_string(frame1), RGBValues);

#if 1
                                             //                int index = 0;
                                             //                for (auto i : pointCloud_1) {
                                             //                    float length = sqrt( i.x*i.x + i.y*i.y + i.z*i.z);
                                             //                    cout<< "HZ:  "<< index << ":  " << i << "   length: " << length << endl;
                                             //                    ++index;
                                             //                }
                                                   std::vector<cv::Point3f> pcloud_CV;
                                                   TriangulateOpenCV(PK_0, PK_LR, points_L1, points_R1, pcloud_CV);

                                             //                index = 0;
                                             //                for (auto i : pcloud_CV) {
                                             //                    float length = sqrt( i.x*i.x + i.y*i.y + i.z*i.z);
                                             //                    cout<< "CV:  "<< index << ":  " << i << "   length: " << length << endl;
                                             //                    ++index;
                                             //                }
                                                             std::vector<cv::Vec3b> RGBValues2;
                                                             for (unsigned int i = 0; i < points_L1.size(); ++i){
                                                                 //uchar grey2 = image_L2.at<uchar>(points_L2[i].x, points_L2[i].y);
                                                                 //RGBValues2.push_back(cv::Vec3b(grey2,grey2,grey2));
                                                                 RGBValues2.push_back(cv::Vec3b(255,0,0));
                                                             }

                                                             AddPointcloudToVisualizer(pcloud_CV, "cloud2" + std::to_string(frame1), RGBValues2);
#endif



          }


      }

      if(image_sub_count == 2)
      {
         image_sub_count = 0;
      }


}

/*void motionEstimation_node::imageCalc(sensor_msgs::ImagePtr &img_left, sensor_msgs::ImagePtr &img_right)
{
       std::vector<cv::Point2f> points_L1, points_R1, points_L2, points_R2;
       refindFeaturePoints(old_img_l, img_left, points_L1_temp, points_L1, points_L2);
       refindFeaturePoints(image_R1, image_R2, points_R1_temp, points_R1, points_R2);

}*/

motionEstimation_node::~motionEstimation_node()
{
}


