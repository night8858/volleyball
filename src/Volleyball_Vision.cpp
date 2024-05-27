#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
// 我记得math库里有定义好的PI，可以直接拿来用
#define _USE_MATH_DEFINES 
#include <cmath>
#define PI M_PI     
// #define PI 3.1415926
using namespace std;

int main()
{
    int MinR = 10;
    int MaxR = 500;
    float maxDistance = 1500.0f; // Maximum distance threshold
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
    // 形态学操作
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
    // 常用变量提前声明
    auto cyan_blue = cv::Scalar(0, 255, 255);
    auto green = cv::Scalar(0, 255, 0);

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

        // 构建形态学操作的结构元
        cv::morphologyEx(binaryImage, binaryImage, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1)); // 闭操作
        cv::morphologyEx(binaryImage, binaryImage, cv::MORPH_OPEN, kernel, cv::Point(-1, -1)); // 开操作
        // imshow("开操作", dst_img);
        cv::findContours(binaryImage, contours, hireachy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < hireachy.size(); i++)
        {
            if (contours[i].size() < 3)
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

            if (ratio < 1.4 && ratio > 0.7)
            // 因为圆的外接直立矩形肯定近似于一个正方形，因此宽高比接近1.0
            {
                printf("X: %f\n", rect.center.x);
                printf("Y: %f\n", rect.center.y);
                printf("圆的面积: %f\n", area);
                printf("圆的半径: %f\n", radius);
                // 如果非debug关闭绘图，这是一个及其耗时的操作
                cv::ellipse(color_image, rect, cyan_blue, 2);
                cv::circle(color_image, rect.center, 2, green, 2, 8, 0);
            }
        }
        // 如果非debug关闭imshow和waitKey，这是一个及其耗时的操作
        cv::imshow("Color Image", color_image);
        cv::imshow("Color", binaryImage);
        cv::waitKey(1);
    }

    return 0;
}
