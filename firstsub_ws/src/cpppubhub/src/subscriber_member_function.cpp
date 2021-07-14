#include <memory> //内存相关

//ros2相关
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;


class MinimalSubscriber : public rclcpp::Node
{
  public:
    /*
    订阅者节点的代码与发布者的代码几乎相同。
    在该节点被命名为minimal_subscriber，
    构造函数使用该节点的create_subscription类来执行回调。
    发布者和订阅者使用的主题名称和消息类型必须匹配，才能进行通信。
    */
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    /*topic_callback函数接收通过主题发布的字符串消息数据，并使用RCLCPP_INFO宏将其写入控制台。*/
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    //订阅
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

/*rclcpp::init初始化ROS 2, 
rclcpp::spin开始处理节点的数据*/
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
