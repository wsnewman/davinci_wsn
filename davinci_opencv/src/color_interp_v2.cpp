//try to interpret images by extracting color
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cwru_srv/simple_int_service_message.h>
#include <opencv2/core/core.hpp>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace cv;
using namespace std;
int var1 = 1;
Point pos;
int radius = 20; //30 covers fiducial
int radius_sample = 20;
int radius_contain = 30;
static const std::string OPENCV_WINDOW = "Image window";
bool g_trigger = false;

//ugly--make images global:
int WIDTH = 640; //set actual values after loading image
int HEIGHT = 480;
cv::Mat image, image_display;
Eigen::Vector3d scene_normalized_avg_color_vec; // avg normalized color vec over entire image

void CallBackFunc(int event, int x, int y, int flags, void* userdata) {
    if (event == EVENT_LBUTTONDOWN) {
        pos.x = x;
        pos.y = y;
        var1 = 0;
        cout << "callback: x,y = " << x << "," << y << endl;
    }
}

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
public:

    ImageConverter()
    : it_(nh_) {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/cameras/left_hand_camera/image", 1,
                &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        if (g_trigger) {
            ROS_INFO("Saving image as snapshot_image.png");
            g_trigger = false;
            cv::Mat &mat = cv_ptr->image; // does this help?
            imwrite("snapshot_image.png", mat);
        }
        // Draw an example circle on the video stream
        if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
            cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));
        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);
        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

bool snapshotService(cwru_srv::simple_int_service_messageRequest& request, cwru_srv::simple_int_service_messageResponse& response) {
    ROS_INFO("snapshot request received");
    response.resp = true; // boring, but valid response info
    g_trigger = true; //signal that we received a request; trigger a response
    return true;
}

// helper fnc: provide x_ctr, y_ctr and half_width (look this many pixels to right and left of ctr);
// provide a normalized color vector
// compute normalized color for each pixel on the row
// do dot product of reference normalized color vector with each pixel and divide by numberof pixels in the row
// a perfect score would be 1.0, meaning every pixel in the row has exactly the same normalized color as the reference
// all other scores will be less than unity

double score_row(int x_ctr, int y_ctr, int half_width, Eigen::Vector3d reference_color_vec) {
    Eigen::Vector3d normalized_color;
    double blue, red, green;
    double color_denom;
    int i_start, i_end;
    double npixels = half_width * 2 + 1;
    i_start = x_ctr - half_width;
    if (i_start < 0) i_start = 0; // avoid running off the left edge of image
    if (i_start > WIDTH) i_start = WIDTH - 1;
    i_end = x_ctr + half_width;
    if (i_end < i_start) i_end = i_start;
    if (i_end > WIDTH - 1) i_end = WIDTH - 1;
    double score = 0.0;
    double pix_score = 0.0;

    for (int icol = i_start; icol < i_end; icol++) {
        for (int icolor = 0; icolor < 3; icolor++) {
            normalized_color[icolor] = image.at<cv::Vec3b>(icol, y_ctr)[icolor]; //             
        }
            //cout << "x,y, colors: " << icol << ", " << y_ctr << ", " << normalized_color.transpose() << endl;
            normalized_color = normalized_color / (normalized_color.norm() + 1);
            normalized_color -= scene_normalized_avg_color_vec;   
            normalized_color = normalized_color/normalized_color.norm();
            pix_score = normalized_color.dot(reference_color_vec);
            //cout<<"pixel score: "<<pix_score<<endl;
            score += pix_score;

    }
    return score / npixels;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle nh; //standard ros node handle
    ros::ServiceServer service = nh.advertiseService("snapshot_svc", snapshotService);
    //cv::namedWindow(OPENCV_WINDOW);
    image = cv::imread("imagel1.png"); //run this pgm from directory containing named images
    image_display = image; // make a copy for mark-up display
    cv::imshow("lcam image 1", image_display);
    Eigen::VectorXd col_rt, col_lft, row_top, row_bot;
    Eigen::Vector3d color_vec, normalized_avg_color_vec, delta_color_vec;
    WIDTH = image.cols;
    HEIGHT = image.rows;
    cout << "width, height = " << WIDTH << ", " << HEIGHT << endl;
    //cv::waitKey(5000);
    int area = (2 * radius + 1)*(2 * radius + 1);
    //Mat frame;
    int i, j;
    //select the region on the image
    while (var1 == 1) {
        //set the callback function for any mouse event
        setMouseCallback("lcam image 1", CallBackFunc, NULL);
        //cap >> frame; // get a new frame from camera
        //show the image
        imshow("lcam image 1", image_display);
        // Wait until user press some key
        waitKey(10);
    }

    cout << "broke out of var1 loop" << endl;
    int npixels = 0;
    int blue = 0;
    int red = 0;
    int green = 0;

    for (int i = 0; i < WIDTH; i++)
        for (int j = 0; j < HEIGHT; j++) {
            npixels++;
            blue += image.at<cv::Vec3b>(i, j)[0]; //(i,j) is x,y = col, row
            green += image.at<cv::Vec3b>(i, j)[1];
            red += image.at<cv::Vec3b>(i, j)[2];
        }
    //red/=npixels;
    //blue/=npixels;
    //green/=npixels;
    //double color_denom = (double) (red+blue+green+1);
    scene_normalized_avg_color_vec[0] = blue;
    scene_normalized_avg_color_vec[1] = green;
    scene_normalized_avg_color_vec[2] = red;
    scene_normalized_avg_color_vec = scene_normalized_avg_color_vec / (scene_normalized_avg_color_vec.norm() + 1);
    cout<<"npixels in entire image: "<<npixels<<endl;
    cout << "sum blue, green, red = " << blue << ", " << green << ", " << red << endl;
    cout << "whole scene normalized avg color vec: " << scene_normalized_avg_color_vec.transpose() << endl;


    //GaussianBlur( frame, frame, Size( 15, 15 ), 0, 0 );imshow("blur",frame);
    //Mat imgHSV,out;
    //cvtColor(frame, imgHSV, CV_BGR2HSV); imshow("HSV",imgHSV);
    //Mat final;
    //final = frame;
    npixels = 0;
    blue = 0;
    red = 0;
    green = 0;
    //compute the color target over sample region about chosen pixel
    for (int var2 = pos.y - radius_sample; var2 <= pos.y + radius_sample; var2++)
        for (int var3 = pos.x - radius_sample; var3 <= pos.x + radius_sample; var3++) {
            npixels++;
            blue += image.at<cv::Vec3b>(var2, var3)[0];
            green += image.at<cv::Vec3b>(var2, var3)[1];
            red += image.at<cv::Vec3b>(var2, var3)[2];
            image_display.at<cv::Vec3b>(var2, var3)[0] = 0;
            image_display.at<cv::Vec3b>(var2, var3)[1] = 0;
            image_display.at<cv::Vec3b>(var2, var3)[2] = 255;
        }

    //red/=npixels;
    //blue/=npixels;
    //green/=npixels;
    //double color_denom = (double) (red+blue+green+1);
    normalized_avg_color_vec[0] = blue;
    normalized_avg_color_vec[1] = green;
    normalized_avg_color_vec[2] = red;
    normalized_avg_color_vec = normalized_avg_color_vec / (normalized_avg_color_vec.norm() + 1);

    cout << "selected patch sum blue, green, red = " << blue << ", " << green << ", " << red << endl;
    cout << "selected patch normalized avg color vec: " << normalized_avg_color_vec.transpose() << endl;
    delta_color_vec= normalized_avg_color_vec-scene_normalized_avg_color_vec;
    delta_color_vec= delta_color_vec/delta_color_vec.norm();
    cout<< "patch delta color vec: "<<delta_color_vec.transpose()<<endl;
    //eval a bunch of row scores:
    double row_score;
    for (int ycol = pos.y; ycol < pos.y + 60; ycol += 10) {
        row_score = score_row(pos.x, ycol, radius_sample, delta_color_vec);
        cout << "ycol,  row score: " << ycol << ", " << row_score << endl;
    }


    while (ros::ok()) {
        cv::imshow("lcam image 1", image_display);
        ros::spinOnce();
        //ros::Duration(0.1).sleep();
        cv::waitKey(30); //need this to allow image display
    }
    //ImageConverter ic;
    //ros::spin();
    return 0;
}

