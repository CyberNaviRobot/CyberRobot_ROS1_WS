#include "ros/ros.h"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>
#include <cstddef>
#include <memory>
#include "ros/wall_timer.h"
#include "pkg06_imu/WitStandardProtocol.h"
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h> // 用于四元数计算


extern std::vector<uint8_t> tx_data_buffer;

class IMU_Node
{
public:
    IMU_Node()
    {
        nh_ = std::make_shared<ros::NodeHandle>("~");
        ROS_INFO("串口节点开启!");

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

        // 创建IMU数据发布者
        imu_pub = nh_->advertise<sensor_msgs::Imu>("imu", 10);

        //定时器500ms发一次
        // transmit_timer_ = nh_->createWallTimer(ros::WallDuration(0.5),
        //                             &IMU_Node::send_data_timer_callback, this);
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

    ~IMU_Node()
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


    ros::Publisher imu_pub;
    ros::WallTimer transmit_timer_;

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
                    //创建imu消息类型
                    auto imu_msg = sensor_msgs::Imu();

                    bool valid_frame = false;  // 添加有效帧标志
                    bool valid_data = false;  // 添加有效帧标志

                    for(int16_t i = 0;i < (int16_t)size;++i)
                    {
                        uint8_t data_buffer = (*read_buffer_)[i];
                        // ROS_DEBUG("接收到的字节：%x",data_buffer);
                        // 处理接收到的数据
                        if(wit_imu_.rx.Data_Analysis(&data_buffer) == true)
                        {
                            valid_frame = true;  // 标记有有效帧需要处理
                        }

                        if(valid_frame == true)
                        {
                            if(wit_imu_.rx.data.cmd == 0x51)
                            {
                            // 存储加速度计数据
                            wit_imu_.rx.data.imu.Accel.X = wit_imu_.rx.data.int16_buffer[0];
                            wit_imu_.rx.data.imu.Accel.Y = wit_imu_.rx.data.int16_buffer[1];
                            wit_imu_.rx.data.imu.Accel.Z = wit_imu_.rx.data.int16_buffer[2];
                            wit_imu_.rx.data.imu.Temp    = wit_imu_.rx.data.int16_buffer[3];
                            
                            //加速度单位m/s2，温度单位℃摄氏度
                            wit_imu_.rx.data.imu.Accel.X = wit_imu_.rx.data.imu.Accel.X / 32768.0f * 16.0f * 9.8f;
                            wit_imu_.rx.data.imu.Accel.Y = wit_imu_.rx.data.imu.Accel.Y / 32768.0f * 16.0f * 9.8f;
                            wit_imu_.rx.data.imu.Accel.Z = wit_imu_.rx.data.imu.Accel.Z / 32768.0f * 16.0f * 9.8f;
                            wit_imu_.rx.data.imu.Temp    =  wit_imu_.rx.data.imu.Temp / 100.0f;

                            // 打印加速度计数据
                            ROS_DEBUG("加速度(XYZ): %.6f, %.6f, %.6f",
                                        wit_imu_.rx.data.imu.Accel.X, wit_imu_.rx.data.imu.Accel.Y, wit_imu_.rx.data.imu.Accel.Z);
                            //减少时间戳带来的延迟
                            imu_msg.header.stamp = ros::Time::now();
                            }

                            else if(wit_imu_.rx.data.cmd == 0x52)
                            {
                            // 存储陀螺仪数据
                            wit_imu_.rx.data.imu.Gyro.X = wit_imu_.rx.data.int16_buffer[0];
                            wit_imu_.rx.data.imu.Gyro.Y = wit_imu_.rx.data.int16_buffer[1];
                            wit_imu_.rx.data.imu.Gyro.Z = wit_imu_.rx.data.int16_buffer[2];
                            
                            //角速度单位rad/s
                            wit_imu_.rx.data.imu.Gyro.X = wit_imu_.rx.data.imu.Gyro.X / 32768.0f * 2000.0f * (M_PI / 180.0f);
                            wit_imu_.rx.data.imu.Gyro.Y = wit_imu_.rx.data.imu.Gyro.Y / 32768.0f * 2000.0f * (M_PI / 180.0f);
                            wit_imu_.rx.data.imu.Gyro.Z = wit_imu_.rx.data.imu.Gyro.Z / 32768.0f * 2000.0f * (M_PI / 180.0f);

                            // 打印陀螺仪数据
                            ROS_DEBUG("角速度(XYZ): %.6f, %.6f, %.6f",
                                        wit_imu_.rx.data.imu.Gyro.X, wit_imu_.rx.data.imu.Gyro.Y, wit_imu_.rx.data.imu.Gyro.Z);
                            }

                            else if(wit_imu_.rx.data.cmd == 0x54)
                            {
                            // 存储磁力计数据(单位lsb)
                            wit_imu_.rx.data.imu.Magnet.X = wit_imu_.rx.data.int16_buffer[0];
                            wit_imu_.rx.data.imu.Magnet.Y = wit_imu_.rx.data.int16_buffer[1];
                            wit_imu_.rx.data.imu.Magnet.Z = wit_imu_.rx.data.int16_buffer[2];
                            
                            //磁场单位uT
                            // wit_imu_.rx.data.imu.Magnet.X = wit_imu_.rx.data.imu.Magnet.X / 1.0f;
                            // wit_imu_.rx.data.imu.Magnet.Y = wit_imu_.rx.data.imu.Magnet.Y / 1.0f;
                            // wit_imu_.rx.data.imu.Magnet.Z = wit_imu_.rx.data.imu.Magnet.Z / 1.0f;

                            // 打印磁力计数据
                            ROS_DEBUG("磁场(XYZ): %.6f, %.6f, %.6f",
                                        wit_imu_.rx.data.imu.Magnet.X, wit_imu_.rx.data.imu.Magnet.Y, wit_imu_.rx.data.imu.Magnet.Z);
                            }

                            else if(wit_imu_.rx.data.cmd == 0x53)
                            {
                            // 存储欧拉角数据
                            wit_imu_.rx.data.imu.Euler.roll = wit_imu_.rx.data.int16_buffer[0];
                            wit_imu_.rx.data.imu.Euler.pitch = wit_imu_.rx.data.int16_buffer[1];
                            wit_imu_.rx.data.imu.Euler.yaw = wit_imu_.rx.data.int16_buffer[2];
                            
                            //欧拉角单位dec
                            wit_imu_.rx.data.imu.Euler.roll = wit_imu_.rx.data.imu.Euler.roll / 32768.0f * 180.0f;
                            wit_imu_.rx.data.imu.Euler.pitch = wit_imu_.rx.data.imu.Euler.pitch / 32768.0f * 180.0f;
                            wit_imu_.rx.data.imu.Euler.yaw = wit_imu_.rx.data.imu.Euler.yaw / 32768.0f * 180.0f;

                            // 打印欧拉角数据
                            ROS_DEBUG("欧拉角(XYZ_RPY): %.6f, %.6f, %.6f",
                                        wit_imu_.rx.data.imu.Euler.roll, wit_imu_.rx.data.imu.Euler.pitch, wit_imu_.rx.data.imu.Euler.yaw);

                            //欧拉角单位rad
                            wit_imu_.rx.data.imu.Euler.roll = wit_imu_.rx.data.imu.Euler.roll * (M_PI / 180.0f);
                            wit_imu_.rx.data.imu.Euler.pitch = wit_imu_.rx.data.imu.Euler.pitch  * (M_PI / 180.0f);
                            wit_imu_.rx.data.imu.Euler.yaw = wit_imu_.rx.data.imu.Euler.yaw  * (M_PI / 180.0f);
                            }

                            else if(wit_imu_.rx.data.cmd == 0x59)
                            {
                            for(int16_t i = 0 ; i < 4 ; i++)
                            {
                                // 存储四元数数据
                                wit_imu_.rx.data.imu.Quat.q[i] = wit_imu_.rx.data.int16_buffer[i];

                                //四元数是归一化的四元数
                                wit_imu_.rx.data.imu.Quat.q[i] = wit_imu_.rx.data.imu.Quat.q[i] / 32768.0f;
                            }

                            // 打印四元数数据
                            //注意，0对应y，1对应x，-2才对应z。
                            ROS_DEBUG("四元数(xyzw): %.6f, %.6f, %.6f, %.6f",
                                        wit_imu_.rx.data.imu.Quat.q[1],wit_imu_.rx.data.imu.Quat.q[0],-wit_imu_.rx.data.imu.Quat.q[2],wit_imu_.rx.data.imu.Quat.q[3]);
                            valid_data = true;
                            }
                            wit_imu_.rx.data.cmd = 0x00;   //跑过一次就进行清0
                            valid_frame = false;
                        }
                    }
                    if(valid_data == true)
                    {
                        //时间戳
                        // imu_msg.header.stamp = ros::Time::now();
                        //位姿信息所参考的坐标系
                        imu_msg.header.frame_id = "imu_link";

                        // tf2::Quaternion q;
                        // //DOF欧拉角单位rad
                        // q.setRPY(roll_, pitch_, yaw_);
                        // //DOF欧拉角（四元数）
                        // imu_msg.orientation.x = q.x();
                        // imu_msg.orientation.y = q.y();
                        // imu_msg.orientation.z = q.z();
                        // imu_msg.orientation.w = q.w();

                        //DOF欧拉角（四元数）
                        //注意对应关系
                        // 传感器坐标系 -> ROS坐标系：
                        // X -> Y
                        // Y -> X
                        // Z -> -Z
                        imu_msg.orientation.x =   wit_imu_.rx.data.imu.Quat.q[1];
                        imu_msg.orientation.y =   wit_imu_.rx.data.imu.Quat.q[0];
                        imu_msg.orientation.z = - wit_imu_.rx.data.imu.Quat.q[2];
                        imu_msg.orientation.w =   wit_imu_.rx.data.imu.Quat.q[3];

                        //加速度
                        imu_msg.linear_acceleration.x = wit_imu_.rx.data.imu.Accel.X;
                        imu_msg.linear_acceleration.y = wit_imu_.rx.data.imu.Accel.Y;
                        imu_msg.linear_acceleration.z = wit_imu_.rx.data.imu.Accel.Z;

                        //角速度
                        imu_msg.angular_velocity.x = wit_imu_.rx.data.imu.Gyro.X;
                        imu_msg.angular_velocity.y = wit_imu_.rx.data.imu.Gyro.Y;
                        imu_msg.angular_velocity.z = wit_imu_.rx.data.imu.Gyro.Z;

                        imu_pub.publish(imu_msg);
                        valid_data = false;
                    }

                }

                // 继续监听新的数据
                asyncReceive();
            }
        );
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
    ros::init(argc, argv, "IMU_Node");  // 初始化节点并指定名称
    auto node = std::make_shared<IMU_Node>();
    ros::spin();       // 主线程处理回调
    return 0;
}
