void 
    ManualRegistration::back_projection_points() {


  right_image_points_ = right_image_.clone();
  left_image_points_ = left_image_.clone();

  pcl::PointXYZRGB minPt, maxPt;
  Eigen::Vector4f centroid = compute_centroid(minPt, maxPt, *cloud_src_points_tr_);

  std::cout << "Centroid: " << centroid << std::endl;
  std::cout << "Max x: " << maxPt.x << std::endl;
  std::cout << "Max y: " << maxPt.y << std::endl;
  std::cout << "Max z: " << maxPt.z << std::endl;
  std::cout << "Min x: " << minPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << std::endl;


  Eigen::Matrix4f tf_left, tf_right;
 
  tf_left <<  camera_mtx_l_msg_.P[0], camera_mtx_l_msg_.P[1], camera_mtx_l_msg_.P[2], camera_mtx_l_msg_.P[3],camera_mtx_l_msg_.P[4], camera_mtx_l_msg_.P[5], camera_mtx_l_msg_.P[6], camera_mtx_l_msg_.P[7], camera_mtx_l_msg_.P[8], camera_mtx_l_msg_.P[9], camera_mtx_l_msg_.P[10], camera_mtx_l_msg_.P[11], 0.0, 0.0, 0.0, 1.0;
  tf_right <<  camera_mtx_r_msg_.P[0], camera_mtx_r_msg_.P[1], camera_mtx_r_msg_.P[2], camera_mtx_r_msg_.P[3],camera_mtx_r_msg_.P[4], camera_mtx_r_msg_.P[5], camera_mtx_r_msg_.P[6], camera_mtx_r_msg_.P[7], camera_mtx_r_msg_.P[8], camera_mtx_r_msg_.P[9], camera_mtx_r_msg_.P[10], camera_mtx_r_msg_.P[11], 0.0, 0.0, 0.0, 1.0;

// /* 
// #  [u v w]' = P * [X Y Z 1]'
// #         x = u / w
// #         y = v / w*/

  for (int  i = 0 ; i < cloud_src_points_tr_->points.size() ; i++) {

    pcl::PointXYZRGB tmp = cloud_src_points_tr_->points[i];    
    
     Eigen::Vector4f dpt1; 
     dpt1 << tmp.x, tmp.y , tmp.z , 1;

     Eigen::Vector4f spt1_r, spt1_l; 
     spt1_r = tf_right*dpt1;
     spt1_l = tf_left*dpt1;

     cv::Point pt1_r((int)(spt1_r(0)/spt1_r(2)), (int)(spt1_r(1)/spt1_r(2)));
     cv::Point pt1_l((int)(spt1_l(0)/spt1_l(2)), (int)(spt1_l(1)/spt1_l(2)));


    { //right
         bool pt1;
         float alpha1;
         pt1 = false;

          if ((pt1_r.x >= 0 && pt1_r.y >= 0) &&  (pt1_r.y < right_image_points_.rows && pt1_r.x < right_image_points_.cols)) {
            if(tmp.z > (2*(maxPt.z - minPt.z)/4)) {
              alpha1 = compute_alpha(minPt, maxPt, tmp.z, 1.0);
              float b = alpha1*(float)255/255.0f + (1-alpha1)*(float)right_image_points_.at<cv::Vec3b>(pt1_r.y,pt1_r.x)[0]/255.0f;
              float g = (float)right_image_points_.at<cv::Vec3b>(pt1_r.y,pt1_r.x)[1]/255.0f;
              float r = (float)right_image_points_.at<cv::Vec3b>(pt1_r.y,pt1_r.x)[2]/255.0f;
              cv::circle( right_image_points_, pt1_r, 4.0, cv::Scalar( b*255.0f,g*255.0f, r*255.0f), -4, 8 );

              //std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
              // std::cout << " alpha is " << alpha1 <<  std::endl;
              // std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;

              pt1 = true;
            }
            else {
              std::cout << "pt1 out of image" << std::endl;
            }
          }
    } //end of right


    { //left
         bool pt1;
         float alpha1;
		 
         pt1 = false;
        
          if ((pt1_l.x >= 0 && pt1_l.y >= 0) &&  (pt1_l.y < left_image_points_.rows && pt1_l.x < left_image_points_.cols)) {
            if(tmp.z > (2*(maxPt.z - minPt.z)/4)) {
              alpha1 = compute_alpha(minPt, maxPt, tmp.z, 1.0);
              float b = alpha1*(float)255/255.0f + (1-alpha1)*(float)left_image_points_.at<cv::Vec3b>(pt1_l.y,pt1_l.x)[0]/255.0f;
              float g = (float)left_image_points_.at<cv::Vec3b>(pt1_l.y,pt1_l.x)[1]/255.0f;
              float r = (float)left_image_points_.at<cv::Vec3b>(pt1_l.y,pt1_l.x)[2]/255.0f;
              cv::circle( left_image_points_, pt1_l, 4.0, cv::Scalar( b*255.0f,g*255.0f, r*255.0f), -4, 8 );
              pt1 = true;
            }
          }
    } //end of left
     
  }
