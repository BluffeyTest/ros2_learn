#include <chrono> //及时相关头文件
#include <functional>
#include <memory> //内存相关头文件
#include <string>

//ros2的头文件，一个是cpp的，一个是消息的
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
// 这个例子创建了Node的一个子类，并使用std::bind()注册一个成员函数作为计时器的回调函数。
class MinimalPublisher : public rclcpp::Node
{
  public:
    /*公共构造函数将节点命名为minimal_publisher，
    并将coun_初始化为0。在构造函数内部，
    使用String消息类型、主题名称topic和
    备份时限制消息所需的队列大小初始化发布者。
    接下来，初始化timer_，这将导致timer_callback函数每秒执行两次。*/
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    /*timer_callback函数是设置消息数据和实际发布消息的地方。
    RCLCPP_INFO宏确保将每个发布的消息打印到控制台。*/
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

/*rclcpp::init初始化ROS 2, 
rclcpp::spin开始处理节点的数据，包括定时器的回调。*/
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
