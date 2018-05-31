#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <laser_geometry/laser_geometry.h>
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <std_srvs/Empty.h>

typedef pcl::PointXYZ                                        Point_type;
typedef pcl::PointCloud<Point_type>                          Point_cloud;

class My_Filter {
     public:
        My_Filter();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void ServoCallback(const sensor_msgs::JointState::ConstPtr& angle);
        void visualizeSinglePC(Point_cloud::Ptr cloud_in);
        bool callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        bool savePC();
        bool add();




     private:
        tf::Transform transf;
        sensor_msgs::PointCloud2 threeD_cloud_;

        Point_cloud pc_ ;

        // Point_cloud::Ptr pc_;
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;
        float prev_rotation;
        float current_rot;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
        ros::Subscriber servo_ang_sub_;
        ros::ServiceServer service;
};

My_Filter::My_Filter(){
        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &My_Filter::scanCallback, this);
        // servo_ang_sub_ = node_.subscribe<std_msgs::Float64> ("/servo", 100, &My_Filter::ServoCallback, this);
        servo_ang_sub_ = node_.subscribe<sensor_msgs::JointState> ("/joint_states", 100, &My_Filter::ServoCallback, this);

        point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/cloud_out", 100, false);
        // service = node_.advertiseService("add_two_ints", savePC);
        service = node_.advertiseService("save_pc", &My_Filter::callback, this);

        prev_rotation = 0.0;
        current_rot = 0.0;
        // initialize service of start rotation

}




void My_Filter::ServoCallback(const sensor_msgs::JointState::ConstPtr& angles){
    current_rot = angles->position[1];

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    static tf::TransformBroadcaster odometry;

    transf.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion q;
    q.setRPY(0.0, (double)(current_rot), 0.0);
    transf.setRotation(q);

    odometry.sendTransform(tf::StampedTransform(transf, header.stamp,
    "laser", "base_link"));

}

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
   

    if ((current_rot - prev_rotation > 0.01) || (current_rot - prev_rotation < -0.01) ) {
         std::cout << " The current angle is= " << current_rot 
        << ", the prev is =" << prev_rotation 
        << " the result is = " << current_rot - prev_rotation << std::endl;
        prev_rotation = current_rot;

        sensor_msgs::PointCloud2 cloud_pc2;
        sensor_msgs::PointCloud2 cloud_rotated;

        projector_.projectLaser(*scan, cloud_pc2);

        pcl_ros::transformPointCloud("laser",transf, cloud_pc2, cloud_rotated);

        Point_cloud::Ptr cloud (new Point_cloud);

        fromROSMsg(cloud_rotated, *cloud);


        // std::cout << "A " << std::endl;
        pc_ += *cloud;
        // std::cout << "B " << std::endl;

        sensor_msgs::PointCloud2 output;
        // std::cout << "C " << std::endl;
        
        pcl::toROSMsg(*cloud, output);
        // pcl::toROSMsg(pc_, output);
        output.header.frame_id = cloud_pc2.header.frame_id;
        std::cout << "Size = " << pc_.points.size() << std::endl;

        // std::cout << "D " << std::endl;

        point_cloud_publisher_.publish(output);
        // std::cout << "E " << std::endl;

        // Point_cloud::Ptr m_ptrCloud(&pc_);

       
    }
}

bool My_Filter::callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new  pcl::PointCloud<pcl::PointXYZRGB>);
   //... populate cloud
 /*   for (size_t i = 0; i < 5; ++i)
    {
        cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }*/

  pcl::visualization::PCLVisualizer viewer4 ("Single PC");
  viewer4.addCoordinateSystem (1.0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> white_color(cloud_in, 0, 255 ,  0);
  
  viewer4.addPointCloud<pcl::PointXYZRGB> (cloud_in, white_color, "cloud_in");
  viewer4.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4.5, "cloud_in");
  while (!viewer4.wasStopped()) viewer4.spinOnce (1);

  /*  std::cout << " Saving point cloud " << std::endl;
    pcl::io::savePCDFileASCII ("/home/joanpep/Desktop/PC.pcd", pc_);
    std::cerr << "PC Saved " << pc_.points.size () << " data points to test_pcd.pcd." << std::endl;*/
    return true;
}

void My_Filter::visualizeSinglePC(Point_cloud::Ptr cloud_in){

    pcl::visualization::PCLVisualizer viewer4 ("Single PC");
    viewer4.addCoordinateSystem (1.0);
    pcl::visualization::PointCloudColorHandlerCustom<Point_type> white_color(cloud_in, 0, 255 ,  0);
    viewer4.addPointCloud<Point_type> (cloud_in, white_color, "cloud_in");
    viewer4.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4.5, "cloud_in");
        while (!viewer4.wasStopped()) viewer4.spinOnce (1);
    }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_filter");
    My_Filter filter;
    ros::spin();

    return 0;
}

