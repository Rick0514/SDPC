
#include <basic.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <icecream.hpp>

inline M3 skew_x(V3 w) {
    M3 w_x;
    w_x << 0, -w(2), w(1),
            w(2), 0, -w(0),
            -w(1), w(0), 0;
    return w_x;
}

inline M3 Exp(V3 w) 
{
    M3 w_x = skew_x(w);

    double theta = w.norm();

    M3 R;

    if(theta == 0)
        R = M3::Identity();
    else
        R = M3::Identity() + (sin(theta) / theta) * (w_x) + ((1 - cos(theta)) / pow(theta,2)) * w_x * w_x;

    return R;
}


class OdomPredictor {
public:
    OdomPredictor(const ros::NodeHandle& nh)
    {
        frame_id_ = "map";
        child_frame_id_ = "imu0";

        pose_.setIdentity();
        linear_velocity_.setZero();
        angular_velocity_.setZero();

        path.header.frame_id = frame_id_;

        imu_sub_ = nh_.subscribe("/imu0", 500, 
            &OdomPredictor::imuCallback, this);

        path_pub_ = nh_.advertise<nav_msgs::Path>("imu_path", 100);
        transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>(
            "imu_transform", 100);
    }

    void imuCallback(const sensor_msgs::ImuConstPtr& msg)
    {
        integrateIMUData(*msg);
        publishOdometry();
        publishTF();
    }

protected:

    void integrateIMUData(const sensor_msgs::Imu& msg)
    {
        if (!has_imu_meas) {
            estimate_timestamp_ = msg.header.stamp;
            has_imu_meas = true;
            return;
        }

        double delta_time = (msg.header.stamp - estimate_timestamp_).toSec();

        V3 imu_linear_acceleration, imu_angular_velocity;
        tf2::fromMsg(msg.linear_acceleration, imu_linear_acceleration);
        tf2::fromMsg(msg.angular_velocity, imu_angular_velocity);

        V3 final_angular_velocity = (imu_angular_velocity - imu_angular_velocity_bias_);
        V3 delta_angle = delta_time * final_angular_velocity / 2.0;
        angular_velocity_ = final_angular_velocity;

        // apply half of the rotation delta
        M3 half_delta_rotation = Exp(delta_angle / 2.0);

        pose_.linear() = pose_.linear() * half_delta_rotation;

        // find changes in linear velocity and position
        V3 delta_linear_velocity =
            delta_time * (imu_linear_acceleration +
                            pose_.rotation().transpose() * kGravity_ -
                            imu_linear_acceleration_bias_);
        pose_.translation() = pose_.translation() +
                pose_.rotation() * delta_time * (linear_velocity_ + delta_linear_velocity / 2.0);
        linear_velocity_ += delta_linear_velocity;

        pose_.linear() = pose_.linear() * half_delta_rotation;
        pose_.linear() = pose_.rotation();

        estimate_timestamp_ = msg.header.stamp;
    }

    void publishOdometry()
    {
        path.header.stamp = estimate_timestamp_;
        
        geometry_msgs::PoseStamped pstd;
        pstd.header = path.header;
        pstd.pose = tf2::toMsg(pose_);

        path.poses.push_back(pstd);
        path_pub_.publish(path);
    }

    void publishTF()
    {
        geometry_msgs::TransformStamped msg;

        msg.header.frame_id = frame_id_;
        msg.header.stamp = estimate_timestamp_;
        msg.child_frame_id = child_frame_id_;

        msg.transform.rotation = tf2::toMsg(Qd(pose_.rotation()));
        msg.transform.translation.x = pose_.translation().x();
        msg.transform.translation.y = pose_.translation().y();
        msg.transform.translation.z = pose_.translation().z();

        transform_pub_.publish(msg);
        br_.sendTransform(msg);
    }

    bool has_imu_meas{false};

    ros::NodeHandle nh_;

    ros::Subscriber imu_sub_;

    ros::Publisher path_pub_;
    ros::Publisher transform_pub_;

    nav_msgs::Path path;
    tf::TransformBroadcaster br_;

    std::string frame_id_;
    std::string child_frame_id_;

    ros::Time estimate_timestamp_;
    
    Pose6 pose_;

    V3 linear_velocity_;
    V3 angular_velocity_;

    V3 imu_linear_acceleration_bias_{0, 0, 0};
    V3 imu_angular_velocity_bias_{0, 0, 0};

    V3 kGravity_{0, 0, -9.8};
    
    bool have_orientation_ = true;
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "imu_to_odom_node");

    ros::NodeHandle nh;
    OdomPredictor odom_predictor(nh);

    ros::spin();
    return 0;
}
