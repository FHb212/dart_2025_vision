// Transporter node: subscribe aim_info and send via USB CDC
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "usb.hpp"

using transporter_sdk::UsbcdcTransporter;

class UsbNode : public rclcpp::Node {
 public:
	UsbNode() : rclcpp::Node("usb_node") {
		// Parameters for device configuration
		vid_ = this->declare_parameter<int>("vid", 0);
		pid_ = this->declare_parameter<int>("pid", 0);
		read_ep_ = this->declare_parameter<int>("read_endpoint", 0x81);   // IN endpoint example
		write_ep_ = this->declare_parameter<int>("write_endpoint", 0x01); // OUT endpoint example
		read_timeout_ = this->declare_parameter<int>("read_timeout", 50);
		write_timeout_ = this->declare_parameter<int>("write_timeout", 50);

		transporter_ = std::make_unique<UsbcdcTransporter>(
				static_cast<unsigned int>(vid_), static_cast<unsigned int>(pid_),
				static_cast<unsigned int>(read_ep_), static_cast<unsigned int>(write_ep_),
				static_cast<unsigned int>(read_timeout_), static_cast<unsigned int>(write_timeout_));

		if (!transporter_->open()) {
			RCLCPP_WARN(this->get_logger(), "USB transporter open failed");
		} else {
			RCLCPP_INFO(this->get_logger(), "USB transporter opened");
		}

		aim_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
				"aim_info", 10,
				[this](const std_msgs::msg::Float32MultiArray &msg) {
					if (msg.data.size() < 3) return;
					transporter::SendPackage pkg{};
					pkg._SOF = RMOS_SEND_ID;
					pkg.ID = 0x1;
					pkg.yaw = msg.data[0];
					pkg.pitch = msg.data[1];
					pkg.is_detectored = (msg.data[2] > 0.5f);
					pkg._EOF = 0xEE;

					int ret = transporter_->write(reinterpret_cast<unsigned char *>(&pkg), sizeof(pkg));
					if (ret < 0) {
						RCLCPP_WARN(this->get_logger(), "USB write failed: %d", ret);
					}
				});
	}

	~UsbNode() override {
		if (transporter_) {
			transporter_->close();
		}
	}

 private:
		int vid_;
		int pid_;
		int read_ep_;
		int write_ep_;
		int read_timeout_;
		int write_timeout_;

		std::unique_ptr<UsbcdcTransporter> transporter_;
		rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr aim_sub_;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<UsbNode>());
	rclcpp::shutdown();
	return 0;
}
