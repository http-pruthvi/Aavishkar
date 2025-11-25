#include <chrono>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "v2v_prototype/msg/basic_safety_message.hpp"
#include "kalman_filter.hpp"

using namespace std::chrono_literals;

class BsmPublisher : public rclcpp::Node {
public:
    BsmPublisher() : Node("bsm_publisher") {
        publisher_ = this->create_publisher<v2v_prototype::msg::BasicSafetyMessage>("bsm", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&BsmPublisher::publish_bsm, this));
        
        // Initialize state (simulating a car moving at 45 degrees)
        kf_.state = {0.0, 0.0, 10.0, 10.0}; // x, y, vx, vy
        
        // Get vehicle ID parameter
        this->declare_parameter("vehicle_id", 1);
        vehicle_id_ = this->get_parameter("vehicle_id").as_int();
    }

private:
    void publish_bsm() {
        // Simulate movement
        double dt = 0.1;
        kf_.predict(dt);
        
        // Create message
        auto msg = v2v_prototype::msg::BasicSafetyMessage();
        msg.id = vehicle_id_;
        msg.latitude = kf_.state[0]; // Using x as lat for demo
        msg.longitude = kf_.state[1]; // Using y as lon for demo
        msg.speed = std::sqrt(kf_.state[2]*kf_.state[2] + kf_.state[3]*kf_.state[3]);
        msg.heading = std::atan2(kf_.state[3], kf_.state[2]);
        msg.acceleration = 0.0;
        msg.brake = 0.0;

        RCLCPP_INFO(this->get_logger(), "Publishing BSM for Vehicle %d: Pos(%.2f, %.2f) Speed(%.2f)", 
            msg.id, msg.latitude, msg.longitude, msg.speed);
            
        publisher_->publish(msg);
    }

    rclcpp::Publisher<v2v_prototype::msg::BasicSafetyMessage>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    KalmanFilter kf_;
    int vehicle_id_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BsmPublisher>());
    rclcpp::shutdown();
    return 0;
}
