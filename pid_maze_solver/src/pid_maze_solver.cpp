#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/twist.hpp"



class PIDMazeSolver : public rclcpp::Node {
public:
    PIDMazeSolver(int scene_number) : Node("turn_controller"), scene_number_(scene_number) {
        pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        sub_ = create_subscription<nav_msgs::msg::Odometry>("/rosbot_xl_base_controller/odom", 10, std::bind(&PIDMazeSolver::odomCallback, this, std::placeholders::_1));
        //rclcpp::sleep_for(std::chrono::seconds(3));
        timer1 = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&PIDMazeSolver::timerCallback1, this));
        count = 0;

        kp_ = 2.0;
        ki_ = 0.0;
        kd_ = 0.0;
        integral_ = 0.0;
        prev_error_ = 0.0;

        integral_lin = 0.0;
        prev_error_lin = 0.0;

        if(scene_number_ == 1){
            angular_targets = {-0.08, -0.60, -0.702400, 0.0016, 0.709600, -0.0208, 0.702400, -0.021600, 0.682279, 1.410279, -0.782521, 1.415879, 1.089479, 1.40};
            
            //, 0.6624, 1.401870, -0.752530};
            
             //-0.082367, -0.816767, -0.092767, -0.428767, -0.176767};

            pose_targets = {
                {0.488832, -0.039190, 0.0},
                {0.703419, -0.185997, 0.0},
                {1.466102, -0.831532, 0.0},
                {1.958901, -0.830743, 0.0},
                {2.353385, -0.491948, 0.0},
                {2.681314, -0.498769, 0.0},
                {3.055634, -0.181946, 0.0},
                {3.635099, -0.194421, 0.0},
                {4.301319,  0.357738, 0.0},
                {4.371142,  0.826039, 0.0},
                {4.580460,  0.576162, 0.0},
                {4.670696,  1.089935, 0.0},
                {4.852022,  1.498381, 0.0},
                {4.909065,  1.878134, 0.0}

                // {4.326364,  0.346297, 0.0},
                // {4.405445,  0.793849, 0.0},
                // {4.601659,  0.610128, 0.0}
                // {3.862752,  0.384650, 0.0},
                // {4.064840,  0.169468, 0.0},
                // {3.517604,  0.220380, 0.0},
                // {3.230208,  0.351757, 0.0},
                // {2.747450,  0.437992, 0.0}
                
            };
        }
        else if(scene_number_ == 2){
            angular_targets = {-0.155506, -0.204931, -0.406195};

            pose_targets = {
                {1.333607, 0.331942, 0.0},
                {1.838251, 0.230313, 0.0}
                
            };
        }
        else{
            RCLCPP_INFO(get_logger(), "Invalid Scene Number");
            rclcpp::shutdown();
        }


    }

private:

    void timerCallback1() {
            target_yaw = angular_targets[count];
            double error = target_yaw - phi;
            integral_ += error;
            derivative = error - prev_error_;
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = kp_ * error + ki_ * integral_ + kd_ * derivative;
            pub_->publish(twist_msg);
            prev_error_ = error;


            if(std::abs(error)<0.001){

                twist_msg.linear.x = 0.0;
                twist_msg.angular.z = 0.0;
                pub_->publish(twist_msg);


                target_x = pose_targets[count][0];
                target_y = pose_targets[count][1];
                target_z = pose_targets[count][2];

                double error_lin = target_x - curr_x;
                integral_lin += error_lin;
                derivative_lin = error_lin - prev_error_lin;
                

                twist_msg.linear.x = kp_ * error_lin + ki_ * integral_lin + kd_ * derivative_lin;
                twist_msg.angular.z = 0.0;
                pub_->publish(twist_msg);
                prev_error_lin = error;


                if(std::abs(error_lin)<0.001){
                    RCLCPP_INFO(get_logger(), "Target: %f, Current: %f", target_yaw, phi);
                    RCLCPP_INFO(get_logger(), "Reached Orientation Waypoint: %d with error: %.3f", (count+1), error);
                    RCLCPP_INFO(get_logger(), "Target: %f, Current: %f", target_x, curr_x);
                    RCLCPP_INFO(get_logger(), "Reached Position Waypoint: %d with error: %.3f", (count+1), error);
                    twist_msg.linear.x = 0.0;
                    twist_msg.angular.z = 0.0;
                    pub_->publish(twist_msg);
                    count++;
                }

                if(count == angular_targets.size()){
                    RCLCPP_INFO(get_logger(), "Execution Complete!");
                    rclcpp::shutdown();
                }
            }

    }


    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        //RCLCPP_INFO(get_logger(), "x: %f, y: %f, z: %f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        //RCLCPP_INFO(get_logger(), "x: %f, y: %f, z: %f, w: %f", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

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
        //RCLCPP_INFO(get_logger(), "R: %f, P:%f, ANGLE: %f", roll, pitch, yaw);
    }


private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    double curr_x = 10000;
    double curr_y = 10000;
    double curr_z = 10000;

    double target_yaw = 0;
    double target_x = 0;
    double target_y = 0;
    double target_z = 0;
    
    geometry_msgs::msg::Twist twist_msg;
    std::vector<std::vector<double>> motions;
    std::chrono::high_resolution_clock::time_point start;
    std::chrono::high_resolution_clock::time_point end;
    int count;
    double phi = 10000;
    std::vector<double> angular_targets;
    std::vector<std::vector<double>> pose_targets;

    rclcpp::TimerBase::SharedPtr timer1, timer2;
    
    double kp_;
    double ki_;
    double kd_;

    double integral_;
    double prev_error_;
    double derivative;

    double integral_lin;
    double derivative_lin;
    double prev_error_lin;

    int scene_number_;
};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int scene_number = 1;
  if (argc > 1) {
    scene_number = std::atoi(argv[1]);
  }
  rclcpp::spin(std::make_shared<PIDMazeSolver>(scene_number));
  rclcpp::shutdown();
}