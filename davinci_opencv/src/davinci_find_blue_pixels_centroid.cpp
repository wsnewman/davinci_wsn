//get an image, search for blue pixels;
// republish on new topic...and print out result
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cwru_msg/vec_of_doubles.h>

static const std::string OPENCV_WINDOW = "Image window2";
using namespace std;

int g_blueratio;
double g_du_right,g_du_left,g_dv_right,g_dv_left;
const double width_in_pixels = 640;
const double height_in_pixels = 240;
const double baseline = 0.010;
const double f = 1034.0; // from horizontal fov = 0.6

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber image_sub_right_;    
    image_transport::Publisher image_pub_;

public:

    ImageConverter()
    : it_(nh_) {
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/davinci/left_camera/image_raw", 1,
                &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video2", 1);

        //extend to right camera:
        image_sub_right_ = it_.subscribe("/davinci/right_camera/image_raw", 1,
                &ImageConverter::imageCbRight, this);
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
        // look for red pixels; turn all other pixels black, and turn red pixels white
        int npix = 0;
        int isum = 0;
        int jsum = 0;
        int redval,blueval,greenval,testval;
        cv::Vec3b rgbpix;
        //image.at<uchar>(j,i)= 255;
        /**/
        for (int i = 0; i < cv_ptr->image.cols; i++)
            for (int j = 0; j < cv_ptr->image.rows; j++) {
                rgbpix = cv_ptr->image.at<cv::Vec3b>(j,i); //[j][i];
                redval = rgbpix[2]+1;
                blueval = rgbpix[0]+1;
                greenval = rgbpix[1]+1;
                testval = blueval/(redval+greenval);
                //redval = (int) cv_ptr->image.at<cv::Vec3b>(j, i)[0]; 
                //cout<<"image("<<j<<","<<i<<")[0] = "<<redval<<endl;
                //if blue, paint it red:
                if (testval > g_blueratio) {
                    cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 0;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 0;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 255;
                    npix++;
                    isum += i;
                    jsum += j;
                }
                else {
                    /*
                    for (int j = 0; j < cv_ptr->image.rows; j++) {
                        cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 128;
                        cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 0;
                        cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 0;
                    }
                     * */
                }
            }
         /* */
        cout << "npix: " << npix << endl;
        if (npix>0) {
            cout << "i_avg: " << isum / npix << endl;
            cout << "j_avg: " << jsum / npix << endl;
        }
        g_du_left= (((double) (isum))/((double) npix))-width_in_pixels/2;
        g_dv_left= (((double) (jsum))/((double) npix))-height_in_pixels/2;

        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());

    }
    
    void imageCbRight(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // look for red pixels; turn all other pixels black, and turn red pixels white
        int npix_right = 0;
        int isum_right = 0;
        int jsum_right = 0;
        int redval,blueval,greenval,testval;
        cv::Vec3b rgbpix;
        //image.at<uchar>(j,i)= 255;
        /**/
        for (int i = 0; i < cv_ptr->image.cols; i++)
            for (int j = 0; j < cv_ptr->image.rows; j++) {
                rgbpix = cv_ptr->image.at<cv::Vec3b>(j,i); //[j][i];
                redval = rgbpix[2]+1;
                blueval = rgbpix[0]+1;
                greenval = rgbpix[1]+1;
                testval = blueval/(redval+greenval);
                //redval = (int) cv_ptr->image.at<cv::Vec3b>(j, i)[0]; 
                //cout<<"image("<<j<<","<<i<<")[0] = "<<redval<<endl;
                //if blue, paint it red:
                if (testval > g_blueratio) {
                    cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 0;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 0;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 255;
                    npix_right++;
                    isum_right += i;
                    jsum_right += j;
                }
                else {
                    /*
                    for (int j = 0; j < cv_ptr->image.rows; j++) {
                        cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 128;
                        cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 0;
                        cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 0;
                    }
                     * */
                }
            }
         /* */
        cout << "npix_right: " << npix_right << endl;
        if (npix_right>0) {
            cout << "i_avg: " << isum_right / npix_right << endl;
            cout << "j_avg: " << jsum_right / npix_right << endl;
        }
        g_du_right= (((double) (isum_right))/((double) npix_right))-width_in_pixels/2;
        g_dv_right= (((double) (jsum_right))/((double) npix_right))-height_in_pixels/2;        

                
        // Update GUI Window
        //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        //cv::waitKey(3);

        // Output modified video stream
        //image_pub_.publish(cv_ptr->toImageMsg());

    }    
};

    int main(int argc, char** argv) {
        ros::init(argc, argv, "blue_triangulator");
        ros::NodeHandle n; // 
        ros::Publisher pub = n.advertise<cwru_msg::vec_of_doubles>("blue_centroid", 1);        
        ImageConverter ic;
        cwru_msg::vec_of_doubles centroid_coords_msg;
        centroid_coords_msg.dvec.resize(3);
        cout<<"enter blue ratio threshold: (e.g. 10) ";
        cin>>g_blueratio;
        ros::Duration timer(0.1);
        double x,y,z;
        double disparity_u;
        //double baseline=0.010; //interocular distance
        //double f = 1032.0; // focal length, in pixels
        while(ros::ok()) {
            disparity_u = g_du_right-g_du_left;
            cout<<"disparity in u: "<<disparity_u<<endl;
            z = baseline*f/disparity_u;
            x = 0.5*((g_du_right + g_du_left)/f)*z -baseline/2;
            y = 0.5*((g_dv_right+g_dv_left)/f)*z;
            ROS_INFO("x,y,z = %f, %f, %f",x,y,z);
            centroid_coords_msg.dvec[0]=x;
            centroid_coords_msg.dvec[1]=y;
            centroid_coords_msg.dvec[2]=z;
            pub.publish(centroid_coords_msg);
            ros::spinOnce();
            timer.sleep();
        }
        return 0;
    }
