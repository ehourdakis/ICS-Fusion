#include "featuredetector.h"
#include<iostream>
#include<fstream>

#include <opencv2/features2d.hpp>
#include"defs.h"
#include <algorithm>

#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>

FeatureDetector::FeatureDetector(kparams_t p, IcsFusion *f, PoseGraph *isam)
    :_fusion(f),
     _params(p),
     _isam(isam)
{

    int  	nfeatures = 500;
    int  	octaveLayers = 3;
    double  contrastThreshold = 0.01;
    double  edgeThreshold = 6;
    double  sigma = 1.6;

    focusThr=20;
    sift = cv::xfeatures2d::SIFT::create(nfeatures,
                                         octaveLayers,
                                         contrastThreshold,
                                         edgeThreshold,
                                         sigma);

    float cov_thr=40;

    covEstim=new SiftCovEstimator(_params.inputSize.y,_params.inputSize.x,cov_thr,true);
    drawnDesc=(uchar3*)malloc(sizeof(uchar3)*_params.inputSize.x*_params.inputSize.y);
    oldDrawnDesc=(uchar3*)malloc(sizeof(uchar3)*_params.inputSize.x*_params.inputSize.y);

    memset(drawnDesc,0,_params.inputSize.x*_params.inputSize.y*3);
    memset(oldDrawnDesc,0,_params.inputSize.x*_params.inputSize.y*3);

    oldFocusMeasure=-1;
}

void FeatureDetector::extractNARFkeypoints(DepthHost &depth,
                                           std::vector<cv::KeyPoint> &keypoints_narf,
                                           std::vector<float3> &keypts3D,
                                           std::vector<FeatDescriptor> &descr)
{
    float support_size = 0.2f;
    int max_no_of_threads = 8;
    float min_interest_value = 0.01;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      
//       convert_cv2pcl_cloud(cloud, pcl_cloud);
//       
//       
//       pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
//       Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
//   // -----------------------------------------------
//   // -----Create RangeImage from the DepthImage-----
//   // -----------------------------------------------
// 
//       std::vector<float> source_depth_data_;
//       int width = cloud.cols, height = cloud.rows;
//       source_depth_data_.resize(width * height);
//       float *depth_buffer = (float *) &source_depth_data_[0];  
// 
//       //std::cout << "Giving colors3\n";
//       for (int i=0; i<width*height; i++) {
//         depth_buffer[i]    = pcl_cloud->points[i].z;
//       }

      float noise_level = 0.0;
      float min_range = 0.0f;
      int border_size = 10;
      
      boost::shared_ptr<pcl::RangeImagePlanar> range_image_ptr (new pcl::RangeImagePlanar);
      pcl::RangeImagePlanar& range_image_planar = *range_image_ptr;   
      
      float center_x = _params.inputSize.x/2;
      float center_y = _params.inputSize.y/2;

      float fx=_params.camera.x;
      float fy=_params.camera.y;
    
      //range_image_planar.setDepthImage (depth.data(), _params.inputSize.x, _params.inputSize.y, center_x, center_y, fx, fy);
      range_image_planar.setDepthImage (depth.data(), _params.inputSize.x, _params.inputSize.y, center_x, center_y, fx, fy);
      range_image_planar.setUnseenToMaxRange();

  // --------------------------------
  // -----Extract NARF keypoints-----
  // --------------------------------
      pcl::RangeImageBorderExtractor range_image_border_extractor;
      pcl::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor);
      narf_keypoint_detector.setRangeImage (&range_image_planar);
      narf_keypoint_detector.getParameters ().support_size = support_size;
      narf_keypoint_detector.getParameters ().max_no_of_threads = max_no_of_threads;
      narf_keypoint_detector.getParameters ().min_interest_value = min_interest_value;
//      narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
      narf_keypoint_detector.getParameters ().calculate_sparse_interest_image = true;
      narf_keypoint_detector.getParameters ().use_recursive_scale_reduction = true;
      
      pcl::PointCloud<int> keypoint_indices;
//       double keypoint_extraction_start_time = pcl::getTime();
      narf_keypoint_detector.compute (keypoint_indices);
//       double keypoint_extraction_time = pcl::getTime()-keypoint_extraction_start_time;
      std::cout << "Found "<<keypoint_indices.points.size ()<<" key points. "<<std::endl;
//               << "This took "<<1000.0*keypoint_extraction_time<<"ms.\n";

     
   
      
      
      // find corresponding index to keypoint 3D coords
      std::vector<cv::Point2f> keypoints_2d;
      std::vector<int> keypoint_indices2;
      keypoint_indices2.resize (keypoint_indices.points.size ());
      for (size_t i=0; i<keypoint_indices.points.size (); ++i)
      {
        int idx=keypoint_indices.points[i];
        
        keypoint_indices2[i]=keypoint_indices.points[i];
      }
    
     
      
    pcl::NarfDescriptor narf_descriptor (&range_image_planar, &keypoint_indices2);
    narf_descriptor.getParameters ().support_size = support_size;
    narf_descriptor.getParameters ().rotation_invariant = false;
    pcl::PointCloud<pcl::Narf36> narf_descriptors;
    narf_descriptor.compute (narf_descriptors);
    std::cout << "Extracted "<<narf_descriptors.size ()<<" descriptors for "
                      <<keypoint_indices.points.size ()<< " keypoints.\n";

    sMatrix4 K=getCameraMatrix(_params.camera);
//     K(0,0)=_params.camera.x;
//     K(1,1)=_params.camera.y;
//     K(0,2)=_params.camera.z;
//     K(1,2)=_params.camera.w;
    
    
    for(int i=0;i<narf_descriptors.size();i++)
    {
        pcl::Narf36 narfd=narf_descriptors[i];
        float3 vert=make_float3(narfd.x,narfd.y,narfd.z);
        
        float3 tmp=rotate(K,vert);
        
        int pix_x = int(tmp.x/tmp.z);
        int pix_y = int(tmp.y/tmp.z);
        
        keypts3D.push_back(vert);
        FeatDescriptor fd;
        
//         std::cout<<vert<<std::endl;
        int idx=keypoint_indices2[i];
        fd.y=pix_x;
        fd.x=pix_y;
        
//         std::cout<<"I:"<<fd.y<<" "<<fd.x<<std::endl;
        memcpy(fd.data,narfd.descriptor,sizeof(float)*36);
        descr.push_back(fd);
        
         cv::Point2f point2d (fd.y,fd.x);
        keypoints_2d.push_back(point2d);
        
    }

    cv::KeyPoint::convert(keypoints_2d, keypoints_narf);
}

Eigen::MatrixXd FeatureDetector::computeCov2DTo3D(Eigen::MatrixXd cov2D,
                                                  double depth,
                                                  double fx,
                                                  double fy,
                                                  double cx,
                                                  double cy,
                                                  double depth_noise_cov)
{

    Eigen::MatrixXd cov3D = Eigen::MatrixXd::Zero(3,3);

    
    //std::cout<<cov2D(0,0)<<" "<<cov2D(0,1)<<std::endl;
    //std::cout<<cov2D(1,0)<<" "<<cov2D(1,1)<<std::endl;

    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(3,2);
    F(0,0) = depth / fx;
    F(1,1) = depth / fy;

    Eigen::MatrixXd L = Eigen::MatrixXd::Zero(3,3);
    L(0,0) = - cx / fx;
    L(1,1) = - cy / fy;
    L(2,2) = 1.00;

    Eigen::MatrixXd Qz = Eigen::MatrixXd::Zero(3,3);
    Qz(2,2) = depth_noise_cov;

    cov3D.noalias() = F*cov2D*F.transpose();

    cov3D.noalias() += L*Qz*L.transpose();

    return cov3D;
}


void FeatureDetector::getFeatImage(uchar3 *out, std::vector<cv::DMatch> &good_matches)
{
    int s=_params.inputSize.x*_params.inputSize.y*3*2;

    if(oldCvKeypoints.size()==0 )
    {
        memset(out,0,s);
        return;
    }

    cv::Mat cvOldDesc(_params.inputSize.y, _params.inputSize.x, CV_8UC3,oldDrawnDesc);
    cv::Mat cvNewDesc(_params.inputSize.y, _params.inputSize.x, CV_8UC3,drawnDesc);

//     good_matches.clear();
    cv::drawMatches( cvRgb, cvKeypoints, oldCvRgb, oldCvKeypoints, good_matches, cvOutput, cv::Scalar::all(-1),
                 cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::DEFAULT );


    memcpy(out,cvOutput.data,s);
}

void FeatureDetector::saveImage(char *filename) const
{
    imwrite( filename, cvOutput );
}

void FeatureDetector::getFeatImage(uchar3 *out)
{    
    covEstim->getDrawnData(out);
}

void FeatureDetector::calcMask(DepthHost &depth,cv::Mat &mask)
{
     mask=cv::Mat::ones(depth.size.y,depth.size.x, CV_8U );

     uint2 px;
     for(px.x=0;px.x<depth.size.x;px.x++)
     {
         for(px.y=0;px.y<depth.size.y;px.y++)
         {
             if(depth[px]<0.0001f ||depth[px]>4.00 || depth[px]!=depth[px])
             {
                 mask.at<uchar>(px.y,px.x,0)=0;
             }
         }
     }
}

void FeatureDetector::detectFeatures(int frame,
                                     DepthHost &depth, RgbHost &rgb,
                                     std::vector<float3> &keypts3D,
                                     std::vector<FeatDescriptor> &descr)
{
    (void) frame;
    keypts3D.clear();
    descr.clear();    

    oldCvRgb=cvRgb;
    cvRgb=cv::Mat(_params.inputSize.y, _params.inputSize.x, CV_8UC3, rgb.data()).clone();
//     cv::Mat cvGrey;
//     cv::cvtColor(cvRgb, cvGrey, CV_BGR2GRAY);


    /*
    cv::Mat cvLaplacian;
    Laplacian(cvGrey, cvLaplacian, CV_8UC1, 3, 1, 0, cv::BORDER_DEFAULT);
    cv::Vec4d m, stdv;
    cv::meanStdDev(cvLaplacian, m, stdv);
    double focusMeasure = (stdv.val[0]*stdv.val[0]) / m.val[0];
    double diff=oldFocusMeasure-focusMeasure;
    if(diff<0)
        diff=-diff;
    std::cout<<"BLUR:"<<focusMeasure<<" "<<diff<<std::endl;


//    char buff[64];
//    sprintf(buff,"/tmp/f%d.png",frame);
//    imwrite( buff, cvLaplacian );

    //if(focusMeasure<focusThr || (oldFocusMeasure>0 && diff>6 )  )
    if(focusMeasure<focusThr )
        return;

    oldFocusMeasure=focusMeasure;
    */


    std::swap(oldDrawnDesc,drawnDesc);
    std::swap(oldCvKeypoints,cvKeypoints);

    cvKeypoints.clear();
    
    extractNARFkeypoints(depth,cvKeypoints,keypts3D,descr);

    std::cout<<"Features Detected:"<<descr.size()<<std::endl;
//     vertexes.release();

//     mask.release();
//     descrMat.release();
//     cvGrey.release();
}
