#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <vector>
#include <string>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;


// ============================================================
// CommandTestNode
//
// 这个 Node 用来模拟：
//
//     C / Coworker
//
// 它不再直接测试 ArbitrationNode，
// 而是模拟 C 按照正式协议发送 /action_intent。
//
// 数据流：
//
// CommandTestNode
//       |
//       | /action_intent
//       v
// CommandingNode
//       |
//       | /algorithm/command/raw
//       v
// ArbitrationNode
// ============================================================
class CommandTestNode : public rclcpp::Node
{
public:

    CommandTestNode()
        : Node("algws_command_test")
    {

        // ====================================================
        // Publisher
        //
        // 模拟 C 向 A Edge 发送正式 action_intent。
        //
        // Topic：
        //
        //     /action_intent
        //
        // 类型：
        //
        //     std_msgs/msg/String
        //
        // String.data 中放 JSON。
        // ====================================================
        publisher_ =
            this->create_publisher<std_msgs::msg::String>(
                "/action_intent",
                10
            );


        RCLCPP_INFO(
            this->get_logger(),
            "📤 C++ Command Test Node started"
        );


        // ====================================================
        // Timer
        //
        // 每 1 秒发送一个测试命令。
        //
        // 这样可以模拟 C 连续发送 action_intent。
        // ====================================================
        timer_ =
            this->create_wall_timer(
                1s,
                std::bind(
                    &CommandTestNode::send_command_sequence,
                    this
                )
            );


        // ====================================================
        // 测试命令
        //
        // 注意：
        //
        // 这里使用的是同事协议中的标准 action。
        //
        // 不再直接使用：
        //
        //     fly_forward_5m
        //
        // 因为 fly_forward_5m 是我们原来
        // Arbitration 测试版本里的本地测试命令，
        // 不是 C 的正式 action_intent action。
        //
        // 正式协议使用：
        //
        //     takeoff
        //     hover
        //     fly_to
        //     track_vel
        //     rtl
        //     land
        //     orbit
        //     turn
        //     stop
        // ====================================================
        commands_ = {

            R"({"action_id":"test_001","action":"takeoff","params":{}})",

            R"({"action_id":"test_002","action":"hover","params":{}})",

            R"({"action_id":"test_003","action":"land","params":{}})"

        };


        index_ = 0;
    }


private:

    // ========================================================
    // Publisher
    // ========================================================
    rclcpp::Publisher<
        std_msgs::msg::String
    >::SharedPtr publisher_;


    // ========================================================
    // Timer
    // ========================================================
    rclcpp::TimerBase::SharedPtr timer_;


    // ========================================================
    // 测试 JSON 命令列表
    // ========================================================
    std::vector<std::string> commands_;


    // ========================================================
    // 当前发送到第几个命令
    // ========================================================
    size_t index_;


    // ========================================================
    // 发送测试命令
    // ========================================================
    void send_command_sequence()
    {

        // ====================================================
        // 如果还有命令没有发送
        // ====================================================
        if (index_ < commands_.size())
        {

            auto msg =
                std_msgs::msg::String();


            // ================================================
            // 把当前 JSON 放进 ROS String
            // ================================================
            msg.data =
                commands_[index_];


            // ================================================
            // 发布到：
            //
            //     /action_intent
            //
            // 这一步模拟 C 发命令给 A Edge。
            // ================================================
            publisher_->publish(msg);


            RCLCPP_INFO(
                this->get_logger(),
                "📤 C -> A: %s",
                msg.data.c_str()
            );


            // 下一个命令
            index_++;
        }

        else
        {

            // =================================================
            // 所有测试命令已经发送完毕
            // =================================================
            RCLCPP_INFO(
                this->get_logger(),
                "✅ 所有 C 模拟测试指令已发送"
            );


            // 停止 Timer
            timer_->cancel();
        }
    }
};


// ============================================================
// main
// ============================================================
int main(
    int argc,
    char * argv[]
)
{

    // 初始化 ROS 2
    rclcpp::init(argc, argv);


    // 创建测试 Node
    auto node =
        std::make_shared<CommandTestNode>();


    // 持续运行 Node
    rclcpp::spin(node);


    // 关闭 ROS 2
    rclcpp::shutdown();


    return 0;
}