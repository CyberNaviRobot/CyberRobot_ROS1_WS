#include "ros/console.h"
#include "ros/ros.h"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>
#include <cstddef>
#include <memory>
#include "ros/subscriber.h"
#include "ros/subscription.h"
#include "ros/wall_timer.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <tf/transform_broadcaster.h>  // TF变换广播（ROS2用tf2_ros）
#include "pkg05_odom_kc/struct_typedef.h"
#include "pkg05_odom_kc/serial_pack.h"


extern std::vector<uint8_t> tx_data_buffer;

class Odom_KC_Node
{
public:
    Odom_KC_Node()
    {
        nh_ = std::make_shared<ros::NodeHandle>("~");
        ROS_INFO("odom节点开启!");

        std::string port = "/dev/ttyUSB0";  //设备名
        int32_t baud_rate = 9600;  //波特率
        int32_t character_size = 8;  // 数据位
        auto stop_bits = boost::asio::serial_port::stop_bits::one;  // 停止位
        auto parity = boost::asio::serial_port::parity::none;  //校验位
        auto flow_control = boost::asio::serial_port::flow_control::none;  //流控制

        //创建串口服务橘饼
        io_service_ = std::make_shared<boost::asio::io_service>();
        serial_port_ = std::make_shared<boost::asio::serial_port>(*io_service_);
        read_buffer_ = std::make_shared<boost::array<uint8_t, 256>>();  // 接收缓冲区

        try
        {
            //设置设备名称
            serial_port_->open(port);
            //设置其他参数
            serial_port_->set_option(boost::asio::serial_port::baud_rate(baud_rate));
            serial_port_->set_option(boost::asio::serial_port::character_size(character_size));
            serial_port_->set_option(boost::asio::serial_port::stop_bits(stop_bits));
            serial_port_->set_option(boost::asio::serial_port::parity(parity));
            serial_port_->set_option(boost::asio::serial_port::flow_control(flow_control));
            
            ROS_INFO("Serial port initialized successfully");
            ROS_INFO("Using device: %s",port.c_str()); // 使用传入的 port 变量作为设备名称
            ROS_INFO("Baud_rate: %d",baud_rate);
        }
        catch (const boost::system::system_error& e)
        {
            ROS_ERROR("配置失败: %s", e.what());
            ros::shutdown();
            return;
        }
        // 创建订阅者
        cmd_vel_sub = nh_->subscribe<geometry_msgs::Twist>("cmd_vel",10,&Odom_KC_Node::cmd_vel_sub_callback,this);

        // 创建发布者（ROS2用create_publisher）
        odom_pub = nh_->advertise<nav_msgs::Odometry>("odom", 10);

        //定时器1ms发一次
        // odom_pub_timer_ = nh_->createWallTimer(ros::WallDuration(0.001),
        //                             &Odom_KC_Node::send_data_timer_callback, this);

        //定时器4ms发一次
        transmit_timer_ = nh_->createWallTimer(ros::WallDuration(0.004),
                                    &Odom_KC_Node::send_data_timer_callback, this);

        //异步接收
        asyncReceive();

        // 启动IO线程
        io_thread_ = std::make_shared<std::thread>(
        [this]()
        {
            ROS_INFO("IO服务线程启动");
            io_service_->run();
            ROS_INFO("IO服务线程退出");
        });
    }

    ~Odom_KC_Node()
    {
        transmit_timer_.stop(); // 停止定时器
    
        // 1. 取消所有异步操作
        if (serial_port_)
        {
            boost::system::error_code ec;
            serial_port_->cancel(ec); // 取消异步操作
            if (ec) // 检查错误码
            {
                ROS_ERROR("取消异步操作失败: %s", ec.message().c_str());
            }
        }

        // 2. 关闭串口
        if (serial_port_ && serial_port_->is_open())
        {
            boost::system::error_code ec;
            serial_port_->close(ec); // 关闭串口
            if (ec) 
            {
                ROS_ERROR("关闭串口失败: %s", ec.message().c_str());
            }
        }

        // 3. 停止IO服务
        if (io_service_)
            io_service_->stop();

        // 4. 等待线程退出
        if (io_thread_ && io_thread_->joinable())
        {
            io_thread_->join();
        }
    }

private:
    // ROS1 节点句柄
    std::shared_ptr<ros::NodeHandle> nh_;

    std::shared_ptr<boost::asio::io_service> io_service_;       // 基础服务
    std::shared_ptr<boost::asio::serial_port> serial_port_;     // 串口对象
    std::shared_ptr<std::thread> io_thread_;                    // IO服务线程

    std::shared_ptr<boost::array<uint8_t, 256>> read_buffer_;   // 接收缓冲区

    ros::Subscriber cmd_vel_sub;
    ros::WallTimer transmit_timer_;

    ros::WallTimer odom_pub_timer_;

    ros::Publisher odom_pub;

    // TF广播器（ROS2用tf2_ros::TransformBroadcaster）
    tf::TransformBroadcaster odom_broadcaster;

    // 四个轮子的速度(对应单片机的顺序)（单位：rpm）
    fp32 received_encoder_wheel_velocities_[4] = {0.0, 0.0, 0.0, 0.0};
    // 四个轮子的速度(对应单片机的顺序)（单位：m/s）
    fp32 encoder_wheel_velocities_[4] = {0.0, 0.0, 0.0, 0.0};

    // 四个轮子的速度(对应单片机的顺序)（单位：2000pc）
    fp32 received_encoder_wheel_angle_[4] = {0.0, 0.0, 0.0, 0.0};

    //麦克纳姆轮底盘参数
    const fp32 wheel_spacing = 0.093; // 轮间距（单位：米）
    const fp32 alex_spacing = 0.085; // 轮距（单位：米）
    const fp32 wheel_radius_ = 0.0375; // 轮子半径（单位：米）

    fp32 x_position_ = 0.0; // x位置
    fp32 y_position_ = 0.0; // y位置
    fp32 yaw_ = 0.0; // 机器人航向角（单位：弧度）
    fp32 dt_;

    fp32 vy,vx,vw;

    struct
    {
        struct
        {
            fp32 x;
            fp32 y;
            fp32 z;
        }linear;
        struct
        {
            fp32 x;
            fp32 y;
            fp32 z;
        }angular;
    }cmd_msg;


    // 共享数据保护
    boost::mutex data_mutex_;


    // 启动异步接收
    void asyncReceive()
    {
        if (!serial_port_->is_open())
        {
            ROS_ERROR("串口未打开，无法接收数据");
            return;
        }

        // 设置异步接收回调函数
        serial_port_->async_read_some(
            boost::asio::buffer(*read_buffer_),  // 使用共享缓冲区
            [this](const boost::system::error_code &ec, std::size_t size)
            {
                ROS_INFO("进入异步回调，状态码: %d", ec.value());
                if (ec) // 检查是否有错误
                {
                    ROS_ERROR("异步接收失败: %s", ec.message().c_str());
                    asyncReceive(); // 错误时重启接收
                    return;
                }

                // 处理接收到的数据
                if (size > 0)
                {
                    ros::Time current_time = ros::Time::now();
                    for (std::size_t i = 0; i < size; ++i)
                    {
                        uint8_t rx_data_buffer = (*read_buffer_)[i];

                        // 处理接收到的数据
                        serial_pack_.rx.Data_Analysis(
                        &rx_data_buffer,
                        0x01,
                        0,
                        0,
                        0,
                        0,
                        8);

                        //由于在ROS1 中，node是局部变量，所以发布方只能在node类里，故Data_Apply不写任何东西，直接在接收下面的回调函数里实现功能。
                        if(serial_pack_.rx.data.cmd == 0x01)
                        {
                            ROS_DEBUG("\n");
                            ROS_DEBUG("以下是电机编码器的数据：");

                            // 存储电机速度数据
                            received_encoder_wheel_velocities_[0] = serial_pack_.rx.data.fp32_buffer[0];  // 电机 0 速度
                            received_encoder_wheel_velocities_[1] = serial_pack_.rx.data.fp32_buffer[1];  // 电机 1 速度
                            received_encoder_wheel_velocities_[2] = serial_pack_.rx.data.fp32_buffer[2];  // 电机 2 速度
                            received_encoder_wheel_velocities_[3] = serial_pack_.rx.data.fp32_buffer[3];  // 电机 3 速度

                            // 存储电机位置数据
                            received_encoder_wheel_angle_[0] = serial_pack_.rx.data.fp32_buffer[4];  // 电机 0 位置
                            received_encoder_wheel_angle_[1] = serial_pack_.rx.data.fp32_buffer[5];  // 电机 1 位置
                            received_encoder_wheel_angle_[2] = serial_pack_.rx.data.fp32_buffer[6];  // 电机 2 位置
                            received_encoder_wheel_angle_[3] = serial_pack_.rx.data.fp32_buffer[7];  // 电机 3 位置

                            vx = serial_pack_.rx.data.fp32_buffer[8];
                            vy = serial_pack_.rx.data.fp32_buffer[9];
                            vw = serial_pack_.rx.data.fp32_buffer[10];
                            yaw_ = serial_pack_.rx.data.fp32_buffer[11];
                            dt_ = serial_pack_.rx.data.fp32_buffer[12];


                            // 打印数据
                            for (int i = 0; i < 4; ++i) 
                            {
                                ROS_DEBUG("%d号电机的速度: %.6f RPM, 位置: %.6f (2000pc)",
                                            i, received_encoder_wheel_velocities_[i], received_encoder_wheel_angle_[i]);

                                ROS_DEBUG("线速度：x:%.6f,y:%.6f,z:%.6f",vx,vy,0.0f);
                                ROS_DEBUG("角速度：x:%.6f,y:%.6f,z:%.6f",0.0f,0.0f,vw);
                                ROS_DEBUG("欧拉角:r:%.6f,p:%.6f,y:%.6f",0.0f,0.0f,yaw_);
                                ROS_DEBUG("积分间隔:%.6f",dt_);
                                
                            }

                            // 创建并填充odom消息
                            nav_msgs::Odometry odom_msg;
                            odom_msg.header.stamp = current_time;
                            odom_msg.header.frame_id = "odom";
                            odom_msg.child_frame_id = "base_link";

                            // 位姿（示例用正弦模拟运动，实际应替换真实数据）
                            odom_msg.pose.pose.position.x = x_position_;
                            odom_msg.pose.pose.position.y = y_position_;
                            odom_msg.pose.pose.position.z = 0.0;

                            //从欧拉角转换为四元数
                            // odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_);
                            tf2::Quaternion q;
                            q.setRPY(0.0, 0.0, yaw_);  // 设置绕XYZ轴的旋转（弧度）
                            odom_msg.pose.pose.orientation.x = q.x();
                            odom_msg.pose.pose.orientation.y = q.y();
                            odom_msg.pose.pose.orientation.z = q.z();
                            odom_msg.pose.pose.orientation.w = q.w();


                            // 线速度
                            odom_msg.twist.twist.linear.x = vx;
                            odom_msg.twist.twist.linear.y = vy;
                            odom_msg.twist.twist.linear.z = 0.0;

                            //角速度
                            odom_msg.twist.twist.angular.x = 0.0;
                            odom_msg.twist.twist.angular.y = 0.0;
                            odom_msg.twist.twist.angular.z = vw;

                            // 发布TF变换
                            geometry_msgs::TransformStamped odom_trans;
                            odom_trans.header.stamp = current_time;
                            odom_trans.header.frame_id = "odom";
                            odom_trans.child_frame_id = "base_link";
                            odom_trans.transform.translation.x = odom_msg.pose.pose.position.x;
                            odom_trans.transform.translation.y = odom_msg.pose.pose.position.y;
                            odom_trans.transform.translation.z = odom_msg.pose.pose.position.z;
                            odom_trans.transform.rotation = odom_msg.pose.pose.orientation;

                            // 发送变换
                            odom_broadcaster.sendTransform(odom_trans);
                            
                            // 发布odom消息
                            odom_pub.publish(odom_msg);
                        }

                    }
                }

                // 继续监听新的数据
                asyncReceive();
            }
        );
    }

    void cmd_vel_sub_callback(const geometry_msgs::Twist::ConstPtr& msg_p)
    {
        boost::mutex::scoped_lock lock(data_mutex_);
        cmd_msg.linear.x = msg_p->linear.x;
        cmd_msg.linear.y = msg_p->linear.y;
        cmd_msg.linear.z = msg_p->linear.z;

        cmd_msg.angular.x = msg_p->angular.x;
        cmd_msg.angular.y = msg_p->angular.y;
        cmd_msg.angular.z = msg_p->angular.z;
    }


    void send_data_timer_callback(const ros::WallTimerEvent& event)
    {
        boost::mutex::scoped_lock lock(data_mutex_);
        // bool bool_buffer[] = {1, 0, 1, 0};
        // int8_t int8_buffer[] = {0x11,0x22};
        // int16_t int16_buffer[] = {2000,6666};
        // // int32_t int32_buffer[] = {305419896};
        // fp32 fp32_buffer[] = {3.5f};
        fp32 fp32_buffer[] = {cmd_msg.linear.x,cmd_msg.linear.y,cmd_msg.linear.z,
                        cmd_msg.angular.x,cmd_msg.angular.y,cmd_msg.angular.z};

        // 发送一串字符串
        // const std::string transmitted_message = "Hello from ROS1 !";
        // auto transmit_data_buffer = std::vector<uint8_t>(transmitted_message.begin(), transmitted_message.end());
        // std::vector<uint8_t> hex_data = {0x48, 0x65, 0x6C, 0x6C, 0x6F}; // "Hello" in ASCII

        if (!serial_port_->is_open())
        {
            ROS_ERROR("串口未打开，无法发送数据");
            return; // 或处理重连
        }

        //由于ROS1中node为局部变量，所以只能在node中调用send函数，所以Serial_Transmit只负责处理data_buffer。
        serial_pack_.tx.Data_Pack(0x01, 
                                    nullptr, 0,
                                    nullptr, 0,
                                    nullptr, 0,
                                    nullptr, 0,
                                    fp32_buffer, sizeof(fp32_buffer) / sizeof(fp32));

        this->asyncSend(tx_data_buffer);
        ROS_DEBUG("平动XYZ：%.6f,%.6f,%.6f", fp32_buffer[0],fp32_buffer[1],fp32_buffer[2]);
        ROS_DEBUG("转动XYZ：%.6f,%.6f,%.6f", fp32_buffer[3],fp32_buffer[4],fp32_buffer[5]);

    }

    //阻塞式发送
    size_t syncSend(const std::vector<uint8_t>& data)
    {
        if(data.empty()) 
        { 
            // 空数据检查
            ROS_WARN("尝试发送空数据");
            return 0;
        }

        try
        {
            // 阻塞直到数据发送完成
            size_t bytes_transferred = boost::asio::write(
                *serial_port_,
                boost::asio::buffer(data)
            );
            return bytes_transferred; // 返回实际发送的字节数
        }
        catch (const boost::system::system_error& e) 
        {
            ROS_ERROR_STREAM("同步发送失败: " << e.what() 
                        << " [错误码: " << e.code() << "]");
            return 0; // 发送失败时返回0
        }
    }

    //异步发送
    void asyncSend(const std::vector<uint8_t>& data)
    {
        auto buf_ptr = std::make_shared<std::vector<uint8_t>>(data);
        boost::asio::async_write(
            *serial_port_,
            boost::asio::buffer(*buf_ptr),
            [this, buf_ptr](const boost::system::error_code& ec, size_t bytes)
            {
                if(ec)
                {
                    ROS_ERROR("异步发送失败: %s", ec.message().c_str());
                    return;
                }
                ROS_DEBUG("异步发送成功(%zu bytes)", bytes);
            }
        );
    }

};

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");  //设置编码
    ros::init(argc, argv, "Odom_KC_Node_node");  // 初始化节点并指定名称
    auto node = std::make_shared<Odom_KC_Node>();
    ros::spin();       // 主线程处理回调
    return 0;
}
