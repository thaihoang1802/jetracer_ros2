#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <iostream>

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>

using namespace boost::placeholders;

#define head1 0xAA
#define head2 0x55 
#define sendType_velocity    0x11
#define sendType_params      0x12
#define sendType_coefficient 0x13 


using boost::asio::io_service; 
using boost::asio::serial_port;
using boost::asio::buffer;

class JetracerNode : public rclcpp::Node {
    public:
        JetracerNode() : Node(" Jetracer_UART_NODE"), io_(), serial_(io_) {

            this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
            this->declare_parameter<int>("baud", 115200);

            this->get_parameter("port", port_name_);
            this->get_parameter("baud", baud_rate_);

            openserial();

            cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,std::bind(&JetracerNode::cmdCallback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(),"Jetracer Node Started");
        }
    private:
    /* Serial */  
        io_service io_;
        serial_port serial_;
        std::string port_name_;
        int baud_rate_;
    /*Velocity*/  
        double x = 0.0; 
        double y = 0.0; 
        double yaw = 0.0; 
    /*Create Subscriber*/
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub; 

    // 1 Check SUM :
        uint8_t checksum(uint8_t* buf, size_t len)
        {
            uint8_t sum = 0x00; 
            for (uint8_t i = 0; i < len; i++)
            {
                sum += *(buf + i); 
            }
            return sum; 
        }
    // 2 Send Velocity: 
        void SetVeclocity(double x, double y, double yaw)
        {
            static uint8_t tmp[11];    
            tmp[0] = head1; 
            tmp[1] = head2;
            tmp[2] = 0x0b; // Lenght = 11 byte
            tmp[3] = sendType_velocity;// Type package Velocity
        // High byte & low byte x 
            tmp[4] = ((int16_t)(x*1000)>>8) & 0xff;
            tmp[5] = ((int16_t)(x*1000)) & 0xff;
        // High byte & low byte y
            tmp[6] = ((int16_t)(y*1000)>>8) & 0xff;
            tmp[7] = ((int16_t)(y*1000)) & 0xff;
        // High byte & low byte Yaw
            tmp[8] = ((int16_t)(yaw*1000)>>8) & 0xff;
            tmp[9] = ((int16_t)(yaw*1000)) & 0xff;
            tmp[10] = checksum(tmp,10);

            boost::asio::write(serial_,buffer(tmp,11));
        }

    // 3 Subscriber Callback 
        void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
            double x = msg->linear.x; 
            double y = msg->linear.y;
            double yaw = msg->angular.z;
            SetVeclocity(x,y,yaw);
        }

    // 4 Serial Open 
        void openserial()
        {
            boost::system::error_code ec; 
            serial_.open(port_name_,ec);
            if (ec)
            {
                RCLCPP_INFO(this->get_logger(),"Failed to open port");
                return ; 
            }
            serial_.set_option(serial_port::baud_rate(115200));
            serial_.set_option(serial_port::flow_control(serial_port::flow_control::none));
            serial_.set_option(serial_port::parity(serial_port::parity::none)); 
            serial_.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
            serial_.set_option(serial_port::character_size(8));
            
            RCLCPP_INFO(this->get_logger(),"Serial Opened!");
        }
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JetracerNode>());
    rclcpp::shutdown();
    return 0 ;
}