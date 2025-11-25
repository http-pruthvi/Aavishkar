#include <memory>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "v2v_prototype/msg/basic_safety_message.hpp"
#include "kalman_filter.hpp"
#include "ttc.hpp"

class BsmListener : public rclcpp::Node {
public:
    BsmListener() : Node("bsm_listener") {
        subscription_ = this->create_subscription<v2v_prototype::msg::BasicSafetyMessage>(
            "bsm", 10, std::bind(&BsmListener::topic_callback, this, std::placeholders::_1));
            
        // My own state (simulated stationary or moving)
        my_state_ = {100.0, 100.0, -10.0, -10.0}; // Moving towards the publisher
        
        this->declare_parameter("vehicle_id", 2);
        my_id_ = this->get_parameter("vehicle_id").as_int();
    }

private:
    void topic_callback(const v2v_prototype::msg::BasicSafetyMessage::SharedPtr msg) {
        if (msg->id == my_id_) return; // Ignore own messages

        // Update peer state
        State peer_state;
        peer_state.x = msg->latitude;
        peer_state.y = msg->longitude;
        peer_state.vx = msg->speed * std::cos(msg->heading);
        peer_state.vy = msg->speed * std::sin(msg->heading);

        // Calculate TTC
        State my_current_state;
        my_current_state.x = my_state_.x;
        my_current_state.y = my_state_.y;
        my_current_state.vx = my_state_.vx;
        my_current_state.vy = my_state_.vy;
        
        // Simulate my movement for the next step
        my_state_.x += my_state_.vx * 0.1;
        my_state_.y += my_state_.vy * 0.1;

        double ttc = calculate_ttc(my_current_state, peer_state);

        if (ttc < 2.0) {
            RCLCPP_WARN(this->get_logger(), "CRITICAL: Collision Warning with Vehicle %d! TTC: %.2f s", msg->id, ttc);
        } else if (ttc < 5.0) {
             RCLCPP_INFO(this->get_logger(), "Caution: Vehicle %d approaching. TTC: %.2f s", msg->id, ttc);
        } else {
             RCLCPP_INFO(this->get_logger(), "Tracking Vehicle %d. Safe.", msg->id);
        }
    }

    rclcpp::Subscription<v2v_prototype::msg::BasicSafetyMessage>::SharedPtr subscription_;
    State my_state_;
    int my_id_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BsmListener>());
    rclcpp::shutdown();
    return 0;
}
