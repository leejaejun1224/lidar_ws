#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <json/json.h>
#include <fstream>

#pragma comment(lib , "json/json_vc71_libmtd")

geometry_msgs::PoseStamped current_pose;
nav_msgs::Path trajectory;
sensor_msgs::Imu imu_msg;

void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    current_pose = *msg;

    geometry_msgs::PoseStamped waypoint;
    waypoint.header.stamp = ros::Time::now();
    waypoint.header.frame_id = "map";


    double yaw = -48.3;
    double radian = yaw * M_PI / 180.0;  
    double x = current_pose.pose.position.x - 41.1;
    double y = current_pose.pose.position.y - 46.29;
    double newX = x * cos(radian) - y * sin(radian);
    double newY = x * sin(radian) + y * cos(radian);
    x = newX;
    y = newY;


    waypoint.pose.position.x = x;//relative_translation.x();
    waypoint.pose.position.y = y;//relative_translation.y();
    waypoint.pose.position.z = 0.0;//relative_translation.z();

    waypoint.pose.orientation.x = 0.0;//relative_rotation.x();
    waypoint.pose.orientation.y = 0.0;//relative_rotation.y();
    waypoint.pose.orientation.z = 0.0;//relative_rotation.z();
    waypoint.pose.orientation.w = 0.0;//relative_rotation.w();



    // waypoint.pose.orientation.x = 0.0;//relative_rotation.x();
    // waypoint.pose.orientation.y = 0.0;//relative_rotation.y();
    // waypoint.pose.orientation.z = 0.0;//relative_rotation.z();
    // waypoint.pose.orientation.w = 0.0;//relative_rotation.w();

    trajectory.poses.push_back(waypoint);


    double yaw2 = msg->pose.orientation.w - 48.3;
    // double yaw = M_PI/2 + theta;
    double cr = cos(0);
    double sr = sin(0);
    double cp = cos(0);
    double sp = sin(0);
    double cy = cos(yaw2*0.5);
    double sy = sin(yaw2*0.5);
    
    double ori_w = cr * cp * cy + sr * sp * sy;
    double ori_x = sr * cp * cy - cr * sp * sy;
    double ori_y = cr * sp * cy + sr * cp * sy;
    double ori_z = cr * cp * sy - sr * sp * cy;   
    // cout << ori_x << endl;
    // cout << ori_y << endl;
    // cout << ori_z << endl;
    // cout << ori_w << endl;

    
    // imu_msg = *msg;
    imu_msg.orientation.x = ori_x;
    imu_msg.orientation.y = ori_y;
    imu_msg.orientation.z = ori_z;
    imu_msg.orientation.w = ori_w;

    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robotz_tf_publisher");
    ROS_INFO("\033[1;32m---->\033[0m Waypoint Tracking Started.");

    ros::NodeHandle n;
    ros::Rate r(0.1);

    tf::TransformBroadcaster broadcaster;
    tf::TransformBroadcaster broadcaster1;

    ros::Publisher trajectory_pub = n.advertise<nav_msgs::Path>("/erp_trajectory", 10);
    ros::Publisher global_path_pub = n.advertise<visualization_msgs::Marker>("/gloabl_path",10);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu/quaternion",10);
    // ros::Publisher lane_pub = n.advertise<visualization_msgs::Marker>("/lane",10);
    ros::Subscriber pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/local_msg", 1, poseCallback);

    while (n.ok()) {
        
        visualization_msgs::Marker global_path;
        visualization_msgs::Marker lane;

        trajectory.header.stamp = ros::Time::now();
        trajectory.header.frame_id = "base_laser";

        Json::Value root;
        Json::Reader reader;
        std::ifstream t;
        std::string index;
        t.open("/home/vision/lidar_ws/src/jaejun/maps/songdo_map.json");
        reader.parse(t, root);
        t.close();

        global_path.scale.x = 0.5;
        global_path.scale.y = 0.5;
        global_path.color.a = 1.0;
        global_path.color.r = 1.0;
        global_path.action = visualization_msgs::Marker::ADD;
        global_path.type = visualization_msgs::Marker::POINTS;
        global_path.header.stamp = ros::Time::now();
        global_path.header.frame_id = "base_laser";
        
        // lane.scale.x = 0.08;
        // lane.scale.y = 0.08;
        // lane.color.a = 1.0;
        // lane.color.g = 1.0;
        // lane.action = visualization_msgs::Marker::ADD;
        // lane.type = visualization_msgs::Marker::POINTS;
        // lane.header.stamp = ros::Time::now();
        // lane.header.frame_id = "base_link";


        for (int k = 0; k < root.size(); ++k) {
            if (k%10 == 0){
                index = std::to_string(k);
                double x = root[index][0].asDouble();
                double y = root[index][1].asDouble();
                double llane_x, llane_y, rlane_x, rlane_y, road_vector, len;
                geometry_msgs::Point node;
                geometry_msgs::Point node_r;
                geometry_msgs::Point node_l;
                node.x = x;
                node.y = y;
                node.z = 0.0;
                
                // if (global_path.points.size() > 0){
                //     road_vector = atan2(fabs(y-global_path.points[-1].y), fabs(x-global_path.points[-1].x));
                //     len = std::hypot(y-global_path.points[-1].y, x-global_path.points[-1].x);
                //     node_l.x = x + len*cos(road_vector) + 2*cos(90-road_vector);
                //     node_l.y = y + len*sin(road_vector) - 2*sin(90-road_vector);
                //     node_l.z = 0.0;
                //     node_r.x = x + len*cos(road_vector) - 2*cos(90-road_vector);
                //     node_r.y = y + len*sin(road_vector) + 2*sin(90-road_vector);
                //     node_r.z = 0.0;
                //     lane.points.push_back(node_l);
                //     lane.points.push_back(node_r);
                // }
                global_path.points.push_back(node);
            }
        }
        global_path_pub.publish(global_path);
        // lane_pub.publish(lane);

        // double angle_deg = current_pose.pose.orientation.w; // 입력 각도 (0에서 360도)
        // double angle_rad = angle_deg * M_PI / 180.0;
        // tf::Quaternion quaternion;
        // quaternion.setRPY(0, 0, 0);
        // // tf::StampedTransform transform(tf::Transform(tf::Quaternion(0,0,0, 1), tf::Vector3(current_pose.pose.position.x , current_pose.pose.position.y, 0.0)),
        // //     ros::Time::now(), "base_link", "base_laser");
        // tf::StampedTransform transform(tf::Transform(tf::Quaternion(quaternion.x(), quaternion.y(),quaternion.z(), 1), tf::Vector3(current_pose.pose.position.x , current_pose.pose.position.y, 0.0)),
        //     ros::Time::now(), "z", "base_laser");    

        // broadcaster.sendTransform(transform);

        // geometry_msgs::PoseStamped waypoint;
        // waypoint.header.stamp = ros::Time::now();
        // waypoint.header.frame_id = "base_link";
        // // base_laser 그러니까 erp 에서의 base_link상 전체 map상에서의 상대적인 궤적
        // tf::Transform relative_transform = transform.inverse();
        // tf::Vector3 relative_translation = relative_transform.getOrigin();
        // tf::Quaternion relative_rotation = relative_transform.getRotation();

        // waypoint.pose.position.x = current_pose.pose.position.x;//relative_translation.x();
        // waypoint.pose.position.y = current_pose.pose.position.y;//relative_translation.y();
        // waypoint.pose.position.z = 0.0;//relative_translation.z();
        // waypoint.pose.orientation.x = 0.0;//relative_rotation.x();
        // waypoint.pose.orientation.y = 0.0;//relative_rotation.y();
        // waypoint.pose.orientation.z = 0.0;//relative_rotation.z();
        // waypoint.pose.orientation.w = 0.0;//relative_rotation.w();

        // trajectory.poses.push_back(waypoint);

        // }
        trajectory_pub.publish(trajectory);
        imu_pub.publish(imu_msg);

        ros::spinOnce();
    }

    return 0;
}
