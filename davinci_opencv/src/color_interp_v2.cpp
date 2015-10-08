//try to interpret images by extracting color
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cwru_srv/simple_int_service_message.h>
#include <opencv2/core/core.hpp>
using namespace cv;
using namespace std;
int var1=1;
Point pos;
int radius = 20; //30 covers fiducial
int radius_sample = 20;
int radius_contain = 30;
static const std::string OPENCV_WINDOW = "Image window";
bool g_trigger=false;
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
if ( event == EVENT_LBUTTONDOWN )
{
pos.x=x;
pos.y=y;
var1=0;
cout<<"callback: x,y = "<<x<<","<<y<<endl;
}
}
class ImageConverter
{
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_;
public:
ImageConverter()
: it_(nh_)
{
// Subscrive to input video feed and publish output video feed
image_sub_ = it_.subscribe("/cameras/left_hand_camera/image", 1,
&ImageConverter::imageCb, this);
image_pub_ = it_.advertise("/image_converter/output_video", 1);
cv::namedWindow(OPENCV_WINDOW);
}
~ImageConverter()
{
cv::destroyWindow(OPENCV_WINDOW);
}
void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
cv_bridge::CvImagePtr cv_ptr;
try
{
cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
}
catch (cv_bridge::Exception& e)
{
ROS_ERROR("cv_bridge exception: %s", e.what());
return;
}
if (g_trigger) {
ROS_INFO("Saving image as snapshot_image.png");
g_trigger=false;
cv::Mat &mat = cv_ptr->image; // does this help?
imwrite("snapshot_image.png",mat);
}
// Draw an example circle on the video stream
if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
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
int main(int argc, char** argv)
{
ros::init(argc, argv, "image_converter");
ros::NodeHandle nh; //standard ros node handle
ros::ServiceServer service = nh.advertiseService("snapshot_svc", snapshotService);
//cv::namedWindow(OPENCV_WINDOW);
cv::Mat image = cv::imread("imagel1.png"); //run this pgm from directory containing named images
cv::imshow("lcam image 1",image);
//cv::waitKey(5000);
int area = (2*radius+1)*(2*radius+1);
//Mat frame;
int i,j;
//select the region on the image
while(var1==1)
{
//set the callback function for any mouse event
setMouseCallback("lcam image 1", CallBackFunc, NULL);
//cap >> frame; // get a new frame from camera
//show the image
imshow("lcam image 1",image);
// Wait until user press some key
waitKey(10);
}
cout<<"broke out of var1 loop"<<endl;
//GaussianBlur( frame, frame, Size( 15, 15 ), 0, 0 );imshow("blur",frame);
//Mat imgHSV,out;
//cvtColor(frame, imgHSV, CV_BGR2HSV); imshow("HSV",imgHSV);
//Mat final;
//final = frame;
int npixels=0;
int blue=0;
int red=0;
int green=0;
//compute the color target over sample region about chosen pixel
for(int var2=pos.y-radius_sample;var2<=pos.y+radius_sample;var2++)
for(int var3=pos.x-radius_sample;var3<=pos.x+radius_sample;var3++)
{
npixels++;
blue+=image.at<cv::Vec3b>(var2, var3)[0];
green+= image.at<cv::Vec3b>(var2, var3)[1];
red+= image.at<cv::Vec3b>(var2, var3)[2];
image.at<cv::Vec3b>(var2, var3)[0] = 0;
image.at<cv::Vec3b>(var2, var3)[1] = 0;
image.at<cv::Vec3b>(var2, var3)[2] = 255;
}
red/=npixels;
blue/=npixels;
green/=npixels;
cout<<"avg red, green, blue = "<<red<<", "<<green<<", "<<blue<<endl;
while(ros::ok()) {
cv::imshow("lcam image 1",image);
ros::spinOnce();
//ros::Duration(0.1).sleep();
cv::waitKey(30); //need this to allow image display
}
//ImageConverter ic;
//ros::spin();
return 0;
}

