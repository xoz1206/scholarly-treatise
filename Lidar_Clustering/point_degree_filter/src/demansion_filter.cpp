#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<vector>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_types.h>
#include<cmath>
#include<string>

class To2D{
public:
    To2D(){
        points_sub = nh.subscribe("/degree_filterd_points", 10000, &To2D::PointsCallback, this);
        filterd_pub = nh.advertise<sensor_msgs::PointCloud2>("/demansion_filterd_points", 10000);
    }

    inline void Down_Sampling(pcl::PointCloud<pcl::PointXYZI>& pd){
        unsigned int point_size = pd.points.size();
        for(unsigned int i = 0; i < point_size; ++i){
            for(unsigned int j = i + 1; j < point_size; ++j){
                if(pd.points[i].x == pd.points[j].x && pd.points[i].y == pd.points[j].y)
                {
                    pd.erase(pd.begin() + j);
                    --j;
                    --point_size;
                }
            }
        }
    }

    void PointsCallback(const sensor_msgs::PointCloud2::ConstPtr& ptr){
        double x, y, z, i;
        sensor_msgs::PointCloud2 points_msg;
        pcl::PointCloud<pcl::PointXYZI> scan, filterd_scan;
        pcl::fromROSMsg(*ptr, scan);
        pcl::fromROSMsg(*ptr, filterd_scan);
        filterd_scan.clear();
        // ROS_INFO("%d\n", scan.points.size());
        // ROS_INFO("%d %d\n", scan.width, scan.height);
    
        for(auto &point : scan.points){
            if(point.z < -1.5) continue;
            point.z = -1.785;
            filterd_scan.points.push_back(point);
            filterd_scan.width = static_cast<uint32_t>(filterd_scan.points.size());
            filterd_scan.height = 1;
        }

        //Down_Sampling(filterd_scan);

        ROS_INFO("points = %ld\n", filterd_scan.points.size());
        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(filterd_scan));
        pcl::toROSMsg(*scan_ptr, points_msg);
        filterd_pub.publish(points_msg);
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber points_sub;
    ros::Publisher filterd_pub;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "demansion_filter");
    To2D T2;
    ros::spin();
    return 0;
}