#include "ros/ros.h"
#include <array>
#include <geometry_msgs/PoseStamped.h>
#include <delta_2/ServoAnglesStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <geometry_msgs/TransformStamped.h>

std::array<std::array<float, 3>, 6> p_p;
std::array<std::array<float, 3>, 6> b_w;

ros::Publisher pub;

float deg2rad(float degree)
{
    float pi = 3.14159265359;
    return (degree * (pi / 180));
}

float rad2deg(float rad)
{
    float pi = 3.14159265359;
    return (rad / pi * 180);
}

float rb;
float rp;
float sb;
float sp;
float ra;
float rs;

float c30 = cos(deg2rad(30));
float s30 = sin(deg2rad(30));

float beta[6] = {deg2rad(30), deg2rad(30), deg2rad(150), deg2rad(150), deg2rad(270), deg2rad(270)};

std::array<float, 6> Theta;

void cb(const geometry_msgs::PoseStamped& msg)
{   
    std::array<float, 3> X;
    X[0] = msg.pose.position.x;
    X[1] = msg.pose.position.y;
    X[2] = msg.pose.position.z;

    tf2::Quaternion q;  
    q.setX(msg.pose.orientation.x);
    q.setY(msg.pose.orientation.y);
    q.setZ(msg.pose.orientation.z);
    q.setW(msg.pose.orientation.w);

    std::array<float, 6> M;
    std::array<float, 6> N;
    std::array<float, 6> Theta;
    std::array<std::array<float, 3>, 6> p_w;
    std::array<std::array<float, 3>, 6> L_w;
    
    for (int i = 0; i < 6; i++)
    {
        // calculate distances from platform and base joints
        tf2::Vector3 p_p_vector3;
        p_p_vector3.setX(p_p[i][0]);
        p_p_vector3.setY(p_p[i][1]);
        p_p_vector3.setZ(p_p[i][2]);       

        tf2::Vector3 p_w0 = tf2::quatRotate(q, p_p_vector3);        

        for (int j = 0; j < 3; j++)
        {
            p_w[i][j] = X[j] + p_w0[j];
            L_w[i][j] = p_w[i][j] - b_w[i][j];
        }
        float rl = sqrt(L_w[i][0]*L_w[i][0] + L_w[i][1]*L_w[i][1] + L_w[i][2]*L_w[i][2]);
        float L = rl*rl - (rs*rs - ra*ra);
        
        // convert distances to servo angles
        M[i] = 2 * ra * p_w[i][2];
        N[i] = 2 * ra * (cos(beta[i]) * (p_w[i][0] - b_w[i][0]) + sin(beta[i]) * (p_w[i][1] - b_w[i][1]));
        float disc = L / sqrt(M[i]*M[i] + N[i]*N[i]);

        // check real solution exists -> disc must be in domain of arcsin(), [-1,1]
        if ((disc >= 1.0) or (disc <= -1.0))
        {
            Theta[i] = nan;
        } else {
            Theta[i] = asin(disc) - atan(N[i] / M[i]);
        }
    }

    if (isnan(Theta[i]))
        {
        ROS_INFO("MANIPULATOR SETPOINT EXCEEDS MATHEMATICAL WORKSPACE")
    } else {
        delta_2::ServoAnglesStamped servo_angles;
        servo_angles.header.frame_id = "servo";
        servo_angles.header.stamp = ros::Time::now();
        for (int i = 0; i < 6; i++)
        {
            servo_angles.Theta[i] = rad2deg(Theta[i]);
        }
        pub.publish(servo_angles);
    }  
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "inverse_kinematics"); 
    ros::NodeHandle nh;

    nh.getParam("/platform_radius", rp);
    nh.getParam("/base_joint_spacing", sb);
    nh.getParam("/platform_joint_spacing", sp);
    nh.getParam("/proximal_link_length", ra);
    nh.getParam("/distal_link_length", rs);

    p_p = {rp * c30 + sp * s30, rp * s30 - sp * c30, 0.0,
                    rp * c30 - sp * s30, rp * s30 + sp * c30, 0.0,
                    -rp * c30 + sp * s30, rp * s30 + sp * c30, 0.0,
                    -rp * c30 - sp * s30, rp * s30 - sp * c30, 0.0,
                    -sp, -rp, 0.0,
                    sp, -rp, 0.0};

    b_w = {rb * c30 + sb * s30, rb * s30 - sb * c30, 0.0,
                    rb * c30 - sb * s30, rb * s30 + sb * c30, 0.0,
                    -rb * c30 + sb * s30, rb * s30 + sb * c30, 0.0,
                    -rb * c30 - sb * s30, rb * s30 - sb * c30, 0.0,
                    -sb, -rb, 0.0,
                    sb, -rb, 0};

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped platform_pos_home;
    platform_pos_home.header.frame_id = "stewart_base";
    platform_pos_home.header.stamp = ros::Time::now();
    platform_pos_home.child_frame_id = "workspace_center";
    platform_pos_home.transform.translation.x = 0.0;
    platform_pos_home.transform.translation.y = 0.0;
    platform_pos_home.transform.translation.z = sqrt(rs*rs - (rb + ra - rp)*(rb + ra - rp));
    platform_pos_home.transform.rotation.x = 0.0;
    platform_pos_home.transform.rotation.y = 0.0;
    platform_pos_home.transform.rotation.z = 0.0;
    platform_pos_home.transform.rotation.w = 1.0;
    static_broadcaster.sendTransform(platform_pos_home);

    ros::Subscriber sub = nh.subscribe("/platform_setpoint/pose", 1, cb);
    pub = nh.advertise<delta_2::ServoAnglesStamped>("/servo_setpoint/positions", 1);
    ros::spin();

    return 0;
}