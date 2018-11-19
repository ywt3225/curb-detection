#include<iostream>
#define PCL_NO_PRECOMPILE
#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<vector>
#include<pcl_ros/point_cloud.h>
#include<pcl/conversions.h>
#include <pcl/kdtree/kdtree_flann.h> 
#include <pcl/surface/mls.h> 
using namespace std;
//自定义点云数据类型pointclloudXYZIR
string cloud_topic_;
struct pointcloudXYZIR
{
  PCL_ADD_POINT4D;                  
  float intensity;
  unsigned short ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
 }EIGEN_ALIGN16; 

POINT_CLOUD_REGISTER_POINT_STRUCT (pointcloudXYZIR, 
                                   (float,x,x)
                                    (float,y,y)
                                    (float,z,z)
                                   (float,intensity,intensity)
                                  (unsigned short,ring,ring)
)

ros::Publisher bat_pub;
void cloudCB(const sensor_msgs::PointCloud2ConstPtr &input)
{  
   pcl::PointCloud<pointcloudXYZIR>::Ptr cloud (new pcl::PointCloud<pointcloudXYZIR>);
  // pcl::PointCloud<pointcloudXYZIR>::Ptr newcloud (new pcl::PointCloud<pointcloudXYZIR>);
   pcl::PointCloud<pointcloudXYZIR>::Ptr newcloud2 (new pcl::PointCloud<pointcloudXYZIR>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr newcloud (new pcl::PointCloud<pcl::PointXYZ> ());
   pcl::fromROSMsg(*input, *cloud);
   float X,Y,Z,I,max,min,z,t;
   unsigned short R,r; 
   std::vector<int> vec;
   std::vector<int> vec2;
   pcl::copyPointCloud(*cloud,*newcloud2);
   for (size_t i = 0; i < cloud->points.size(); i++) 
   { 

   cloud->points[i].z=0;
   if(R<=3)
   vec.push_back(i);
   }
   pcl::copyPointCloud(*cloud,vec,*newcloud);
   pcl::io::savePCDFile("3333.pcd",*newcloud);
   
  /* 
   while(R<=3)
       {
       
       if(max<Z) max=Z;
       else if(min>Z) min=Z;
       }
   }
       t=(max-min)/10;
   for (size_t i = 0; i < cloud->points.size(); i++) 
   { 
   X = cloud->points[i].x;
   Y = cloud->points[i].y; 
   Z = cloud->points[i].z;
   I = cloud->points[i].intensity;
   R = cloud->points[i].ring;
       if (R<=3 && Z>min+2*t && Z<max-2*t)
          vec.push_back(i);
   } 
   pcl::copyPointCloud(*cloud,vec,*newcloud);
   pcl::io::savePCDFile("2222.pcd",*newcloud);
   max=newcloud->points[0].z;
   min=newcloud->points[0].z;
   cout<<max<<endl;
   for (size_t j = 0; j < newcloud->points.size(); j++) 
   {
   z = newcloud->points[j].z;
   r = newcloud->points[j].ring;
      {
       if (max<z) 
          max=z;
       else if(min>z)
          min=z;
      }
      t=(max-min)/10;
      cout<<t<<endl;
      if (z>(min+2*t) && z<(max-2*t))
          vec2.push_back(j);
   }
   pcl::copyPointCloud(*newcloud,vec2,*newcloud2);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointNormal> mls_points;
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setComputeNormals (true); 
  mls.setInputCloud (newcloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.1);
  mls.process (mls_points);*/
   
   pcl::io::savePCDFile("2222.pcd",*newcloud);
   while (ros::ok())
   {    
   sensor_msgs::PointCloud2 output;
   pcl::toROSMsg(*newcloud, output);
   output.header.frame_id = "velod";
   ros::Rate loop_rate(1000);
   bat_pub.publish(output);
   ros::spinOnce();
   loop_rate.sleep();
   } 
}
main (int argc, char **argv)
{  
   ros::init (argc, argv, "velodyne_points_filter");
   ros::NodeHandle nh;
   cloud_topic_ = "input";
   ros::Subscriber bat_sub = nh.subscribe(cloud_topic_, 1, cloudCB); 
   
   bat_pub = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points_2", 1);
   ros::spin();
   return 0;
}

