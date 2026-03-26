#include <rclcpp/rclcpp.hpp>
#include <service_interfaces/srv/setpos.hpp>

class HandControlClient : public rclcpp::Node {
public:
    HandControlClient()
        : Node("hand_control_client") {
        // 创建服务客户端
        client_ = this->create_client<service_interfaces::srv::Setpos>("Setpos");
    }

    void send_request() {
        // 等待服务可用
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for the service to be available...");
        }

        // 创建请求和响应
        auto request = std::make_shared<service_interfaces::srv::Setpos::Request>();

        // 设置请求参数，确保在范围内
        request->pos0 = 0;
        request->pos1 = 0;
        request->pos2 = 0;
        request->pos3 = 0;
        request->pos4 = 0;
        request->pos5 = 0;

        // 发送请求
        auto result_future = client_->async_send_request(request);

        // 等待结果
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Service response received: %s", 
                        result_future.get()->pos_accepted ? "Positions Accepted" : "Positions Rejected");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get a response from the service.");
        }
    }

private:
    rclcpp::Client<service_interfaces::srv::Setpos>::SharedPtr client_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<HandControlClient>();
    client_node->send_request(); // 发送请求
    rclcpp::shutdown();
    return 0;
}

