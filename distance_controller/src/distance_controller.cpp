#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/twist.hpp"



class DistanceController : public rclcpp::Node {
public:
    DistanceController() : Node("distance_controller") {
        pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        sub_ = create_subscription<nav_msgs::msg::Odometry>("/rosbot_xl_base_controller/odom", 10, std::bind(&DistanceController::odomCallback, this, std::placeholders::_1));
        //rclcpp::sleep_for(std::chrono::seconds(3));
        timer1 = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&DistanceController::timerCallback1, this));
        count = 0;

        kp_ = 1.0;
        ki_ = 0.0;
        kd_ = 0.0;
        integral_ = 0.0;
        prev_error_ = 0.0;

        targets = {
            {1.0, 0.0, 0.0},
            {2.0, 0.0, 0.0},
            {3.0, 0.0, 0.0}
        };


    }

private:

    void timerCallback1() {

            target_x = targets[count][0];
            target_y = targets[count][1];
            target_z = targets[count][2];

            double error = target_x - curr_x;

            //RCLCPP_INFO(get_logger(), "Error: %f", error);

            integral_ += error;
            derivative = error - prev_error_;


            twist_msg.linear.x = kp_ * error + ki_ * integral_ + kd_ * derivative;
            twist_msg.angular.z = 0.0;
            pub_->publish(twist_msg);

            if(std::abs(error)<0.001){
                RCLCPP_INFO(get_logger(), "Target: %f, Current: %f", target_x, curr_x);
                RCLCPP_INFO(get_logger(), "Reached waypoint: %d with error: %.3f", (count+1), error);
                twist_msg.linear.x = 0.0;
                twist_msg.angular.z = 0.0;
                pub_->publish(twist_msg);
                count++;
            }

            if(count == targets.size()){
                RCLCPP_INFO(get_logger(), "Execution Complete!");
                rclcpp::shutdown();
            }

    }


    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        //RCLCPP_INFO(get_logger(), "x: %f, y: %f, z: %f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

        curr_x = msg->pose.pose.position.x;
        curr_y = msg->pose.pose.position.y;
        curr_z = msg->pose.pose.position.z;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x, 
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, 
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        phi = yaw;
    }


private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    double curr_x = 10000;
    double curr_y = 10000;
    double curr_z = 10000;

    double target_x = 0;
    double target_y = 0;
    double target_z = 0;
    geometry_msgs::msg::Twist twist_msg;
    std::vector<std::vector<double>> motions;
    std::chrono::high_resolution_clock::time_point start;
    std::chrono::high_resolution_clock::time_point end;
    int count;
    double phi;
    std::vector<std::vector<double>> targets;
    rclcpp::TimerBase::SharedPtr timer1, timer2;
    
    double kp_;
    double ki_;
    double kd_;

    double integral_;
    double prev_error_;

    double derivative;
    double control_signal;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceController>());
    rclcpp::shutdown();
    return 0;
}