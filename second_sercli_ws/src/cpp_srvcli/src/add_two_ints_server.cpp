//ros2相关的头文件
#include "rclcpp/rclcpp.hpp"

//相关的文件夹下面好像并没有找到这个hpp，只有对应的.srv
#include "example_interfaces/srv/add_two_ints.hpp"

//内存相关
#include <memory>

//从.srv文件搁置中按要求获得request，和response，并打印到命令行
void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
{
  response->sum = request->a + request->b;  //!约定运算方式
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);  //!看起来是获得要求的数据
  //在命令行输出运算的结果
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

/******************************************************************************
 * @brief 主函数
 * @param  argc             My Param doc
 * @param  argv             My Param doc
 * @return int 
 ******************************************************************************/
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);//!初始化

  //创建node并命名
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  //创建服务，这个也和依赖相关性太大了，这个反正也太麻烦了
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

  //打印准备信息
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  //绑定节点
  rclcpp::spin(node);
  rclcpp::shutdown();
}
