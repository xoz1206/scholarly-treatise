#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <queue>
#include <vector>

//사용자 헤더
#include "clustering/find_road_points.h"
//메시지 헤더
#include "lidar_detect/object_height.h"

using namespace pcl;
using namespace std;

class Plane
{
    public:
        Plane() : cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>)
        {   
            sub = nh.subscribe("/points_raw", 100, &Plane::callback, this);
            pub4 = nh.advertise<sensor_msgs::PointCloud2>("/projected_cloud", 10);
            height_pub = nh.advertise<lidar_detect::object_height>("/object_height_topic", 100);
        }

        void callback(const sensor_msgs::PointCloud2Ptr &ptr)
        {
            sensor_msgs::PointCloud2 point_msg;
        
            pcl::fromROSMsg(*ptr, scan);
            pcl::fromROSMsg(*ptr, filterd_scan);
            filterd_scan.clear();

            pcl::VoxelGrid<pcl::PointXYZ> vg;

            vg.setInputCloud(scan.makeShared());//scan PointCloud data copy
            vg.setLeafSize(0.15f,0.15f,0.15f);//set the voxel grid size //15cm
            vg.filter(*cloud_filtered);//create the filtering object

            make_plane_RANSAC();
            projection_onto_plane();
        }

        void make_plane_RANSAC()
        {
            int count = 0;

            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            // Optional
            seg.setOptimizeCoefficients (true);
            // Mandatory
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (0.01);

            pcl::PointCloud<pcl::PointXYZ> filtered_points_cloud_z;
            
            // z 값이 filtering 된 point들을 가지고 pointcloud 만드는 작업. RANSAC 알고리즘에 넣어주기 위해
            for(int k = 0; k < cloud_filtered->points.size(); ++k)
            {
                if(fabs(cloud_filtered->points[k].x) < 10 && fabs(cloud_filtered->points[k].y) < 10 && cloud_filtered->points[k].z < -1.5)
                {
                    pcl::PointXYZ z_filtered_point;
                    z_filtered_point.x = cloud_filtered->points[k].x;
                    z_filtered_point.y = cloud_filtered->points[k].y;
                    z_filtered_point.z = cloud_filtered->points[k].z;
                    filtered_points_cloud_z.push_back(z_filtered_point);
                    count++;
                }
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr point_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_points_cloud_z));
            seg.setInputCloud (point_ptr);
            seg.segment (*inliers, *coefficients);

            if (inliers->indices.size () == 0)
            {
                PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            }

            normal_vector.x = coefficients->values[0];
            normal_vector.y = coefficients->values[1];
            normal_vector.z = coefficients->values[2];
            D = (-1)*coefficients->values[3];
            normal_vector_queue.push(normal_vector);
            D_queue.push(D);
            extract_normal_vector(); // normal_vector의 n개의 평균을 구한다.
            cout << " total_z_points : " << count << endl;
        }

        void extract_normal_vector()
        {
            if(normal_vector_queue.size() == normal_vector_queue_size && D_queue.size() == normal_vector_queue_size)
            {
                float sum_x = 0.0;
                float sum_y = 0.0;
                float sum_z = 0.0;
                float sum_d = 0.0;

                for(int k = 0; k<normal_vector_queue.size(); ++k)
                {
                    sum_x += normal_vector_queue.front().x;
                    sum_y += normal_vector_queue.front().y;
                    sum_z += normal_vector_queue.front().z;
                    sum_d += D_queue.front();
                }
                //다시 갱신.
                normal_vector.x = sum_x / normal_vector_queue.size();
                normal_vector.y = sum_y / normal_vector_queue.size();
                normal_vector.z = sum_z / normal_vector_queue.size();
                D = sum_d /D_queue.size();

                cout << " normal_vector_average : " << normal_vector.x << " , " << normal_vector.y << " , " << normal_vector.z << " , " << D << endl;
                
                //최신의 4개의 data point를 가지고 평균을 내기때문
                normal_vector_queue.pop(); //맨 앞 원소 제거
                D_queue.pop();
            }
            else
            {
                cout << "normal_vector_queue's size and D_queue.'s size is under " << normal_vector_queue_size <<
 endl;
            }
        }
        void projection_onto_plane()
        {
            Eigen::Vector4f coeffs;
            coeffs << normal_vector.x, normal_vector.y, normal_vector.z, -D;

            pcl::PointCloud<pcl::PointXYZI> projected_cloud_pcl;

            for(size_t i = 0; i < cloud_filtered->points.size(); ++i)
            {
                // projection이 수행되어야 하는 영역안의 points 추출 후, projection
                if(fabs(cloud_filtered->points[i].x) < x_limit && fabs(cloud_filtered->points[i].y) < y_limit && cloud_filtered->points[i].z < z_high_limit &&  cloud_filtered->points[i].z > z_low_limit )
                {
                    pcl::PointXYZI projection;

                    projection.x = cloud_filtered->points[i].x; 
                    projection.y = cloud_filtered->points[i].y;
                    projection.z = (-1) * (normal_vector.x * cloud_filtered->points[i].x + normal_vector.y * cloud_filtered->points[i].y - D) / normal_vector.z;
                    
                    projection.intensity = 2.0;

                    //projection을 넣어줄 때 그 해당하는 cloud_filtered의 높이 정보를 전달하기 위해 pointcloud data vector 순서와 동일하게 값을 넣는다.
                    msg.object_height.push_back(cloud_filtered->points[i].z);
                }
            }

            sensor_msgs::PointCloud2 projected_cloud;
            pcl::PointCloud<pcl::PointXYZI>::Ptr n_ptr(new pcl::PointCloud<pcl::PointXYZI>(projected_cloud_pcl));
            pcl::toROSMsg(*n_ptr, projected_cloud);

            projected_cloud.header.frame_id = "velodyne";
            pub4.publish(projected_cloud);
            cout << msg.object_height.size()<< endl;
            height_pub.publish(msg);
            msg.object_height.resize(0);
        }
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Publisher pub4;
        ros::Publisher height_pub;
        ros::Subscriber sub;

        // 높이 정보를 위한 msg
        lidar_detect::object_height msg;

        geometry_msgs::Point normal_vector; //법선벡터
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
        pcl::PointCloud<pcl::PointXYZ> scan, filterd_scan;
        // 급격한 변화를 없애기 위해
        queue< geometry_msgs::Point > normal_vector_queue; 
        queue< float > D_queue;

        float D; // 평면의 방정식의 상수 값
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "RANSAC_plane");
    Plane p;
    ros::spin();
}