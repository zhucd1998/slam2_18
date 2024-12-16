#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include "../../../include/System.h"

using namespace std;
using namespace std::chrono;

class ImageGrabber
{
public:
ImageGrabber(ORB_SLAM2::System* pSLAM) : mpSLAM(pSLAM), first_message_received(false) {}

void GrabImage(const sensor_msgs::ImageConstPtr& msg);

ORB_SLAM2::System* mpSLAM;
time_point<steady_clock> last_msg_time = steady_clock::now();
bool first_message_received; // 标记是否接收到第一条消息
};

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // 标记已接收到第一条消息
    first_message_received = true;

    // 更新最后接收消息的时间
    last_msg_time = steady_clock::now();

    // 将ROS消息转换为OpenCV格式的图像
    cv_bridge::CvImageConstPtr cv_ptr;
    try
        {
            cv_ptr = cv_bridge::toCvShare(msg);
        }
    catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

    // 使用转换后的图像调用 SLAM 系统的单目追踪函数
    mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);

    ros::Rate rate(10);  // 主循环频率
    const auto timeout = seconds(10);  // 超时时间设为10秒

    // 主循环，等待消息并检查是否超时
    while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();

            // 如果还没有接收到第一条消息，则跳过超时检查
            if (!igb.first_message_received)
                continue;

            // 检查当前时间与最后接收消息时间的间隔
            auto now = steady_clock::now();
            if (now - igb.last_msg_time > timeout)
            {
                std::cout << "No new messages received for 10 seconds. Exiting..." << std::endl;
                break;
            }
        }

    // 循环退出后，停止 SLAM 系统的所有线程并保存关键帧轨迹
    SLAM.Shutdown();

    SLAM.SaveKeyFrameTrajectoryTUM("/home/zcd/ORB_SLAM2/KeyFrameTrajectory.txt");
    std::cout << "KeyFrameTrajectory saved." << std::endl;

    ros::shutdown();
    return 0;
}
