#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <laser_geometry/laser_geometry.h>
#include "sensor_msgs/JointState.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>

#include <std_srvs/Empty.h>

typedef pcl::PointXYZ                                        Point_type;
typedef pcl::PointCloud<Point_type>                          Point_cloud;

class cloudGenerator {
public:
    cloudGenerator();
    
    void visualizeSinglePC(Point_cloud::Ptr cloud_in);
    void tf2Matrix4x4(tf::Transform tf_in, Eigen::Matrix4f& transform_out);
    //-- Topics Callbacks
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void ServoCallback(const sensor_msgs::JointState::ConstPtr& angle);
    //--Services Callbacks
    bool callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool clearPCCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);



private:
    ros::NodeHandle node_;
    tf::Transform transf_;
    Point_cloud pc_ ;
    laser_geometry::LaserProjection projector_;
    float prev_rotation_;
    float current_rot_;
    float th_rot_;

    ros::Publisher point_cloud_publisher_;
    ros::Subscriber scan_sub_;
    ros::Subscriber servo_ang_sub_;
    ros::ServiceServer service;
    ros::ServiceServer service_clear_pc;

};

cloudGenerator::cloudGenerator() {
    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/input_scan", 100, &cloudGenerator::scanCallback, this);
    servo_ang_sub_ = node_.subscribe<sensor_msgs::JointState> ("/joint_states", 100, &cloudGenerator::ServoCallback, this);

    point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/cloud_out", 100, false);
    service = node_.advertiseService("save_pc", &cloudGenerator::callback, this);
    service_clear_pc = node_.advertiseService("clear_pc", &cloudGenerator::clearPCCallback, this);

    th_rot_ = 0.0052;
    prev_rotation_ = 0.0;
    current_rot_ = 0.0;
}

//--Services
bool cloudGenerator::clearPCCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) 
{
    pc_.clear();
    std::cout << "Global PC clear" << std::endl;
}

bool cloudGenerator::callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) 
{
    std::cout << " Saving point cloud " << std::endl;
    pcl::io::savePCDFileASCII ("/home/joanpep/Desktop/PC.pcd", pc_);
    std::cerr << "PC Saved " << pc_.points.size () << " data points as PC.pcd" << std::endl;
    return true;
}

//-- Topics Callback
void cloudGenerator::ServoCallback(const sensor_msgs::JointState::ConstPtr& angles) {
    current_rot_ = angles->position[1];

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    static tf::TransformBroadcaster odometry;

    transf_.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion q;
    q.setRPY((double)(current_rot_), 0.0, 0.0);
    transf_.setRotation(q);

    odometry.sendTransform(tf::StampedTransform(transf_, header.stamp,
                           "laser", "base_link"));

}

void cloudGenerator::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {


    if ((current_rot_ - prev_rotation_ > th_rot_) || (current_rot_ - prev_rotation_ < -th_rot_) ) {
        std::cout << " The current angle is= " << current_rot_
                  << ", the prev is =" << prev_rotation_
                  << " the result is = " << current_rot_ - prev_rotation_ << std::endl;
        prev_rotation_ = current_rot_;

        sensor_msgs::PointCloud2 cloud_pc2;
        sensor_msgs::PointCloud2 cloud_rotated;

        projector_.projectLaser(*scan, cloud_pc2);

        pcl_ros::transformPointCloud("laser", transf_, cloud_pc2, cloud_rotated);

        Point_cloud::Ptr cloud (new Point_cloud);
        Point_cloud::Ptr transformed_cloud (new Point_cloud);


        fromROSMsg(cloud_rotated, *cloud);
        Eigen::Matrix4f tr_matrix;

        tf2Matrix4x4(transf_, tr_matrix);
        pcl::transformPointCloud (*cloud, *transformed_cloud, tr_matrix);

        pc_ += *transformed_cloud;

        sensor_msgs::PointCloud2 output;

        pcl::toROSMsg(pc_, output);
        output.header.frame_id = cloud_pc2.header.frame_id;
        std::cout << "Size = " << pc_.points.size() << std::endl;

        point_cloud_publisher_.publish(output);
    }
}


void cloudGenerator::tf2Matrix4x4(tf::Transform tf_in, Eigen::Matrix4f& transform_out)
{

    tf::Quaternion quat = tf_in.getRotation();
    tf::Matrix3x3 rotation (quat);

    tf::Vector3 translation = tf_in.getOrigin();
    transform_out = Eigen::Matrix4f::Identity();

    transform_out(0, 0) = rotation[0][0];
    transform_out(0, 1) = rotation[0][1];
    transform_out(0, 2) = rotation[0][2];

    transform_out(1, 0) = rotation[1][0];
    transform_out(1, 1) = rotation[1][1];
    transform_out(1, 2) = rotation[1][2];

    transform_out(2, 0) = rotation[2][0];
    transform_out(2, 1) = rotation[2][1];
    transform_out(2, 2) = rotation[2][2];

    transform_out(0, 3) = translation[0];
    transform_out(1, 3) = translation[1];
    transform_out(2, 3) = translation[2];
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_filter");

    cloudGenerator filter;
    ros::spin();

    return 0;
}

