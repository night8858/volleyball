#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#define PI 3.1415926
using namespace cv;
using namespace std;
int main() {
    int MinR = 10; int MaxR = 500;
    float maxDistance = 1500.0f; // Maximum distance threshold

    // Create a RealSense pipeline
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
     int width = 640;  // 图像的宽度  
    int height = 480; // 图像的高度  
    cv::Mat blackImage = cv::Mat::zeros(height, width, CV_8UC1); // 创建一个全黑的彩色图像
    rs2::pipeline_profile profile = pipe.start(cfg);

    // Create a align object
    rs2::align align_to_color(RS2_STREAM_COLOR);

    while (true) {
        // Wait for the next set of frames
        rs2::frameset frames = pipe.wait_for_frames();

        // Align the frames
        auto aligned_frames = align_to_color.process(frames);

        // Get aligned depth frame
        rs2::depth_frame depth_frame = aligned_frames.get_depth_frame();

        // Get color frame
        rs2::video_frame color_frame = aligned_frames.first(RS2_STREAM_COLOR);

        // Convert color frame to OpenCV format
        cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // Iterate over the depth data
        const uint16_t* depth_data = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
        int width = depth_frame.get_width();
        int height = depth_frame.get_height();
        int depth_pitch = depth_frame.get_stride_in_bytes() / sizeof(uint16_t);
        Mat image=color_image.clone();
        for (int x = 0; x < height; ++x) {
            for (int y = 0; y < width; ++y) {
                int index = x * depth_pitch + y;
                uint16_t depth_value = depth_data[index];
                if (depth_value < maxDistance) { // Only add points within maximum distance threshold
                    // Add point with distance information to the point set
                    image.at<cv::Vec3b>(x, y) = cv::Vec3b(255, 255, 255);
                }
                if(depth_value > 1700){
                    image.at<cv::Vec3b>(x, y) = cv::Vec3b(0, 0, 0);
                }
            }
        }
        Mat grayImage;
        cvtColor(image, grayImage, COLOR_BGR2GRAY);
        Mat binaryImage;
        threshold(grayImage, binaryImage, 220, 255, THRESH_BINARY);
        GaussianBlur(binaryImage, binaryImage, Size(9, 9), 2, 2);
		vector<vector<Point>> contours;	
		vector<Vec4i> hireachy;
		// 形态学操作	
		Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
		// 构建形态学操作的结构元	
		morphologyEx(binaryImage, binaryImage, MORPH_CLOSE, kernel, Point(-1, -1));
		//闭操作	
		kernel = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
		// 构建形态学操作的结构元
		morphologyEx(binaryImage, binaryImage, MORPH_OPEN, kernel, Point(-1, -1));
		//开操作	
		//imshow("开操作", dst_img);
		findContours(binaryImage, contours, hireachy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
  		for (int i = 0; i < hireachy.size(); i++)
		{
			if (contours[i].size() < 3)continue;
			double area = contourArea(contours[i]);
			if (area < 20)continue;
				//晒选出轮廓面积大于2000的轮廓	
			double arc_length = arcLength(contours[i], true);
			double radius = arc_length / (2 * PI);
	
			if (!(MinR < radius && radius < MaxR))
			{
				continue;
			}
		
			RotatedRect rect = fitEllipse(contours[i]);
			float ratio = float(rect.size.width) / float(rect.size.height);
		
			if (ratio < 1.4 && ratio > 0.7)
				//因为圆的外接直立矩形肯定近似于一个正方形，因此宽高比接近1.0	
			{
                printf("X: %f\n", rect.center.x);
				printf("Y: %f\n", rect.center.y);
				printf("圆的面积: %f\n", area);
				printf("圆的半径: %f\n", radius);
				ellipse(color_image, rect, Scalar(0,255, 255),2);
				circle(color_image, rect.center, 2, Scalar(0,255,0), 2, 8, 0);
			}
        }
        cv::imshow("Color Image", color_image);
        cv::imshow("Color", binaryImage);
        cv::waitKey(1);
    }

    return 0;
}
