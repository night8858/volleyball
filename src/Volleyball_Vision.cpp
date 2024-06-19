#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <iostream>
#include <pthread.h>

#include <termios.h>
#include <boost/asio.hpp>
// 我记得math库里有定义好的PI，可以直接拿来用
#define _USE_MATH_DEFINES
#include <cmath>
#define PI M_PI
// #define PI 3.1415926
using namespace std;


#include <iostream>

////////////低通滤波的类////////////
class LowPassFilter {
public:
    LowPassFilter(double dt, double RC) : dt(dt), RC(RC), prev_y(0), first_run(true) {}

    double filter(double x) {
        if (first_run) {
            first_run = false;
            prev_y = x;
        }
        double alpha = dt / (RC + dt);
        double y = alpha * x + (1 - alpha) * prev_y;
        prev_y = y;
        return y;
    }

private:
    double dt;
    double RC;
    double prev_y;
    bool first_run;
};
//////////////低通滤波的类///////////

///////////// 串口 通信类 /////////////
class SerialPort
{
public:
    uint8_t send_date[12];

    typedef struct
    {
        float x;
        float y;
        float deep;
        uint8_t ball_flag;
    } Ball_tracking_Pos;

    /// 初始化数据结构体/////
    Ball_tracking_Pos ball_tracking_pos; // 真实球的位置
    // Ball_tracking_Pos ball_tracking_prediction_pos; // 预测球的位置

    SerialPort(boost::asio::io_service &io, const std::string &port)
        : serial(io, port)
    {
        serial.set_option(boost::asio::serial_port_base::baud_rate(115200)); // 设置波特率
    }

    void send_message(const std::string &message)
    {
        boost::asio::write(serial, boost::asio::buffer(message));
    }

private:
    boost::asio::serial_port serial;
};
///////////// 串口 通信类 /////////////


//声明//

//限制数据范围函数
float limit_data(float date , float min, float max);
void Float_to_Byte(float a, float b, float c, unsigned char byte[]);


int main()
{
    int MinR = 70;
    int MaxR = 700;
    float maxDistance = 2000.0f; // Maximum distance threshold
    int width = 640;             // 图像的宽度
    int height = 480;            // 图像的高度

    // realsense2
    // Create a RealSense pipeline
    rs2::pipeline pipe;
    rs2::config cfg;
    // cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 30);
    // cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);
    // 可以尝试更改设置里的帧数
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 0);
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 0);

    rs2::pipeline_profile profile = pipe.start(cfg);
    // Create a align object
    rs2::align align_to_color(RS2_STREAM_COLOR);

    // 声明前置，不影响处理
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hireachy;
    // opencv
    // 声明前置，不影响处理
    cv::Mat grayImage;
    cv::Mat binaryImage;
    cv::Mat blackImage = cv::Mat::zeros(height, width, CV_8UC1); // 创建一个全黑的彩色图像

    // 定义蓝色的范围
    cv::Scalar lower_blue(110, 50, 50);
    cv::Scalar upper_blue(160, 255, 255);

    // 定义黄色的范围
    cv::Scalar lower_yellow(25, 100, 100);
    cv::Scalar upper_yellow(60, 255, 255);

    // 形态学操作
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
    // 常用变量提前声明
    auto cyan_blue = cv::Scalar(0, 255, 255);
    auto green = cv::Scalar(0, 255, 0);

    LowPassFilter lpf(0.1, 1.0);  // 创建滤波器，你可以根据需要调整dt和RC的值

    boost::asio::io_service io; // 调用IO口
    // SerialPort sp(io, "/dev/ttyS0"); // 测试图像用
    SerialPort sp(io, "/dev/ttyACM0"); // 请根据实际情况修改串口设备路径

    while (true)
    {
        // Wait for the next set of frames
        rs2::frameset frames = pipe.wait_for_frames();

        // Align the frames
        auto aligned_frames = align_to_color.process(frames);

        // Get aligned depth frame
        rs2::depth_frame depth_frame = aligned_frames.get_depth_frame();

        // Get color frame
        rs2::video_frame color_frame = aligned_frames.first(RS2_STREAM_COLOR);

        // Convert color frame to OpenCV format
        cv::Mat color_image(cv::Size(width, height), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // Iterate over the depth data
        // 我用realsence相机用的少，深度图的长款应该都是固定的吧，可以不用每一次都获取一下
        const uint16_t *depth_data = reinterpret_cast<const uint16_t *>(depth_frame.get_data());
        int _width = depth_frame.get_width();
        int _height = depth_frame.get_height();
        int depth_pitch = depth_frame.get_stride_in_bytes() / sizeof(uint16_t);
        // 这里是不是可以在不显示的情况下不用克隆，复制数据是一个很耗时的操作
        cv::Mat image = color_image.clone();
        for (int x = 0; x < _height; ++x)
        {
            for (int y = 0; y < _width; ++y)
            {
                int index = x * depth_pitch + y;
                uint16_t depth_value = depth_data[index];
                if (depth_value < maxDistance)
                {
                    // Only add points within maximum distance threshold
                    // 仅添加最大距离阈值内的点
                    // Add point with distance information to the point set
                    // 将带有距离信息的点添加到点集中
                    image.at<cv::Vec3b>(x, y) = cv::Vec3b(255, 255, 255);
                }
                if (depth_value > 1700)
                {
                    image.at<cv::Vec3b>(x, y) = cv::Vec3b(0, 0, 0);
                }
            }
        }

        cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
        cv::threshold(grayImage, binaryImage, 220, 255, cv::THRESH_BINARY);
        cv::GaussianBlur(binaryImage, binaryImage, cv::Size(9, 9), 2, 2);

            // 计算总像素数
        int totalPixels = grayImage.total();
        // 计算白色和黑色像素数
        int whitePixels = cv::countNonZero(grayImage);

        // 计算白色像素占比
        float whiteRatio = (float)whitePixels / (float)totalPixels;

        // 构建形态学操作的结构元
        cv::morphologyEx(binaryImage, binaryImage, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1)); // 闭操作
        cv::morphologyEx(binaryImage, binaryImage, cv::MORPH_OPEN, kernel, cv::Point(-1, -1));  // 开操作
        // imshow("开操作", dst_img);
        cv::findContours(binaryImage, contours, hireachy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        // 遍历轮廓
        for (int i = 0; i < hireachy.size(); i++)
        {
            if (contours[i].size() < 5)
                continue;
            double area = cv::contourArea(contours[i]);
            if (area < 20)
                continue;
            // 晒选出轮廓面积大于2000的轮廓
            double arc_length = cv::arcLength(contours[i], true);
            double radius = arc_length / (2 * PI);

            if (!(MinR < radius && radius < MaxR))
            {
                continue;
            }

            cv::RotatedRect rect = cv::fitEllipse(contours[i]);
            float ratio = float(rect.size.width) / float(rect.size.height);

            ///////////////  颜色检测部分  ///////////////
            // 创建掩码
            cv::Mat mask = cv::Mat::zeros(color_image.size(), CV_8UC1);
            cv::ellipse(mask, rect, cv::Scalar(255), -1);

            // 应用掩码
            cv::Mat img_roi;
            cv::bitwise_and(color_image, color_image, img_roi, mask);

            // 颜色空间转换
            cv::Mat hsv;
            cv::cvtColor(img_roi, hsv, cv::COLOR_BGR2HSV);

            cv::Mat blue_mask, yellow_mask;
            cv::inRange(hsv, lower_blue, upper_blue, blue_mask);
            cv::inRange(hsv, lower_yellow, upper_yellow, yellow_mask);

            //  判断是否含有蓝色和黄色
            bool has_blue = cv::countNonZero(blue_mask) > 20;
            bool has_yellow = cv::countNonZero(yellow_mask) > 20;

            ///////////////  颜色检测部分  ///////////////

            if (ratio < 1.6 && ratio > 0.7)
            // 因为圆的外接直立矩形肯定近似于一个正方形，因此宽高比接近1.0
            {
                if (has_blue == true && has_yellow == true)
                {
                    //sp.ball_tracking_pos.x = lpf.filter(rect.center.x);
                    //sp.ball_tracking_pos.y = lpf.filter(rect.center.y);
                    // 发送数据
                    sp.ball_tracking_pos.x = limit_data(rect.center.x , 1 , 640 );
                    sp.ball_tracking_pos.y = limit_data(rect.center.y , 0 , 480 );
                    sp.ball_tracking_pos.deep = depth_frame.get_distance(sp.ball_tracking_pos.x , sp.ball_tracking_pos.y );

                    if(whiteRatio > 0.94) {sp.ball_tracking_pos.deep = 0.15;}  //判断球到脸上无法识别的情况

                    Float_to_Byte(sp.ball_tracking_pos.x, sp.ball_tracking_pos.y, sp.ball_tracking_pos.deep, sp.send_date);//转float

                    std::string message(reinterpret_cast<char *>(sp.send_date), sizeof(sp.send_date));

                    printf("X: %f\n", sp.ball_tracking_pos.x);

                    printf("Y: %f\n", sp.ball_tracking_pos.y);

                    printf("D: %f\n", sp.ball_tracking_pos.deep);
                    sp.send_message(message);
                    // 如果非debug关闭绘图，这是一个及其耗时的操作
                    cv::ellipse(color_image, rect, cyan_blue, 2);
                    cv::circle(color_image, rect.center, 2, green, 2, 8, 0);
                }
                else
                {
                    sp.ball_tracking_pos.x = 320;
                    sp.ball_tracking_pos.y = 240;
                    sp.ball_tracking_pos.deep = 0;
                    if(whiteRatio > 0.94) {sp.ball_tracking_pos.deep = 0.15;}
                    Float_to_Byte(sp.ball_tracking_pos.x, sp.ball_tracking_pos.y, sp.ball_tracking_pos.deep, sp.send_date);

                    std::string message(reinterpret_cast<char *>(sp.send_date), sizeof(sp.send_date));
                    printf("未识别到球\n");
                    sp.send_message(message);
                }
            }
            else
            {
                sp.ball_tracking_pos.x = 320;
                sp.ball_tracking_pos.y = 240;
                if(whiteRatio > 0.94) {sp.ball_tracking_pos.deep = 0.15;}
                Float_to_Byte(sp.ball_tracking_pos.x, sp.ball_tracking_pos.y, sp.ball_tracking_pos.deep, sp.send_date);

                std::string message(reinterpret_cast<char *>(sp.send_date), sizeof(sp.send_date));
                printf("未识别到球\n");
                sp.send_message(message);
            }
        }
        // 如果非debug关闭imshow和waitKey，这是一个及其耗时的操作
        printf("whiteRatio: %f\n", whiteRatio);
        cv::imshow("Color Image", color_image);
        //cv::imshow("Color", binaryImage);
        cv::waitKey(1);
    }

    return 0;
}

// 限制数据范围
float limit_data(float date , float min, float max)
{
    if (date < min)
    {
        date = min;
    }
    if (date > max)
    {
        date = max;
    }
    return date;
}

typedef union
{
    float fdata;
    unsigned long ldata;
} FloatLongType;

/*
将浮点数f转化为4个字节数据存放在byte[4]中
*/
void Float_to_Byte(float a, float b, float c, unsigned char byte[])
{
    FloatLongType fl, f2 ,f3;   
    fl.fdata = a;
    f2.fdata = b;
    f3.fdata = c;
    byte[0] = (unsigned char)fl.ldata;
    byte[1] = (unsigned char)(fl.ldata >> 8);
    byte[2] = (unsigned char)(fl.ldata >> 16);
    byte[3] = (unsigned char)(fl.ldata >> 24);
    byte[4] = (unsigned char)f2.ldata;
    byte[5] = (unsigned char)(f2.ldata >> 8);
    byte[6] = (unsigned char)(f2.ldata >> 16);
    byte[7] = (unsigned char)(f2.ldata >> 24);
    byte[8] = (unsigned char)f3.ldata;
    byte[9] = (unsigned char)(f3.ldata >> 8);
    byte[10] = (unsigned char)(f3.ldata >> 16);
    byte[11] = (unsigned char)(f3.ldata >> 24);
}


    