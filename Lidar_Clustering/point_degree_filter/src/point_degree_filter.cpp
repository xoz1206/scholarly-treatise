#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<vector>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_types.h>
#include<cmath>
#include<string>

constexpr static double PI = 3.14159265359;
const double radi = PI / 180.0;
class Degreefilter{
public:
    Degreefilter() : sideDegree(0.0), x_filter(0.0), z_filter(1.5), max_range(25.0){
        // if(!nh.getParam("/side_degree", sideDegree))    throw std::runtime_error("set side_degree!");
        // if(!nh.getParam("/x_filter", x_filter))         throw std::runtime_error("set x_filter!");
        // if(!nh.getParam("/z_filter", z_filter))         throw std::runtime_error("set z_filter!");
        //if(!nh.getParam("/input_topic", input_topic))   throw std::runtime_error("set input_topic!");
        //if(!nh.getParam("/output_topic", output_topic)) throw std::runtime_error("set output_topic!");
        side_filter = 1 / tan(sideDegree * radi);
        side_filter2 = 1 / tan((90 - sideDegree) * radi);
        ROS_INFO("==========PARAMS_INFO==============");
        //ROS_INFO("Subscribe   : %s" , input_topic.c_str());
        //ROS_INFO("Publish     : %s" , output_topic.c_str());
        ROS_INFO("side_degree : %lf", sideDegree);
        ROS_INFO("x_filter    : %lf", x_filter);
        ROS_INFO("z_filter    : %lf", z_filter);
        ROS_INFO("PI          : %lf", PI);
        ROS_INFO("radi        : %lf", radi);
        ROS_INFO("side_filter : %lf", side_filter);
        ROS_INFO("  cos(degree) : %lf", cos(sideDegree * radi));
        ROS_INFO("  sin(degree) : %lf", sin(sideDegree * radi));
        ROS_INFO("  tan(degree) : %lf", tan(sideDegree * radi));
        ROS_INFO("==================================");

        point_sub = nh.subscribe("/filtered_points", 10000, &Degreefilter::PointsCallback, this);
        filterd_pub = nh.advertise<sensor_msgs::PointCloud2>("/degree_filtered_points", 10000);
    }

    void PointsCallback(const sensor_msgs::PointCloud2::ConstPtr& ptr){
        double x, y, z, i;
        sensor_msgs::PointCloud2 filtered_msg;
        pcl::PointCloud<pcl::PointXYZI> scan, filterd_scan;
        pcl::fromROSMsg(*ptr, scan);
        pcl::fromROSMsg(*ptr, filterd_scan);
        filterd_scan.clear();

        for(auto &point : scan.points){
            // if(point.x * point.x + point.y * point.y > max_range * max_range) continue;
            //if(!(fabs(point.y) < side_filter*point.x)) continue; // only front
            if(point.z < -1.5) continue;
            if(point.x > max_range || point.x < -max_range) continue;
            if(point.y > max_range || point.y < -max_range) continue;
            if(point.z > z_filter) continue; // velodyne tf is 1.785. So 3.285m z filtering
            if(!(fabs(point.y) < side_filter*fabs(point.x))) continue; // remove side
            point.z = -1.785;
            filterd_scan.points.push_back(point);
            filterd_scan.width = static_cast<uint32_t>(filterd_scan.points.size());
            filterd_scan.height = 1;
        }
        //ROS_INFO("points = %ld\n", filterd_scan.points.size());
        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(filterd_scan));
        pcl::toROSMsg(*scan_ptr, filtered_msg);
        filterd_pub.publish(filtered_msg);
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber point_sub;
    ros::Publisher filterd_pub;
    std::string input_topic, output_topic;
    double sideDegree, max_range;
    double x_filter, z_filter, side_filter, side_filter2;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "point_degree_filter");
    Degreefilter df;

    ros::spin();
    return 0;
}