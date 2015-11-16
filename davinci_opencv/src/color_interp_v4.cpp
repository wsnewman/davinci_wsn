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
#include <string.h>
#include <fstream>





using namespace cv;
using namespace std;
int var1 = 1;
Point pos;
//search params for red ball
int radius = 40;// use 50 for "h" scenes; use 40 for "L" scenes; 
int radius_sample = radius;
int radius_contain = radius*1.5;
int radius_search = radius*3;

//default values: change these, as appropriate
int WIDTH = 640; //set actual values after loading image
int HEIGHT = 480;
int NPICS = 24; // assumes 24 pictures
int row_ctr = HEIGHT/2;
int col_ctr_left= WIDTH/2;
int col_ctr_rt = WIDTH/2;
const double g_baseline = 0.005; //baseline horizontal separation of cameras, in meters
const double g_focal_length = 1034.0; // focal length, in pixels

double NOM_BALL_RED= 137.0;
double NOM_BALL_GREEN = 62.0;
double NOM_BALL_BLUE = 46.0;


static const std::string OPENCV_WINDOW = "Image window";
bool g_trigger = false;

char g_image_fname[64];
double g_x_centroid_left[64];
double g_y_centroid_left[64];
double g_x_centroid_right[64];
double g_y_centroid_right[64];

double g_dist[64];
double g_x_world[64];
double g_y_world[64];

//ugly--make images global:
cv::Mat image, image_display_left,image_display_right,image_markup;
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
    int col_start, col_end;
    int row = y_ctr;
    double npixels = half_width * 2 + 1;
    col_start = x_ctr - half_width;
    if (col_start < 0) col_start = 0; // avoid running off the left edge of image
    if (col_start > WIDTH) col_start = WIDTH - 1;
    col_end = x_ctr + half_width;
    if (col_end < col_start) col_end = col_start;
    if (col_end > WIDTH - 1) col_end = WIDTH - 1;
    double score = 0.0;
    double pix_score = 0.0;

    for (int col = col_start; col < col_end; col++) {
        for (int icolor = 0; icolor < 3; icolor++) {
            normalized_color[icolor] = image.at<cv::Vec3b>(row, col)[icolor]; //             
        }
            //cout << "x,y, colors: " << icol << ", " << y_ctr << ", " << normalized_color.transpose() << endl;
            normalized_color = normalized_color / (normalized_color.norm() + 1);
            normalized_color -= scene_normalized_avg_color_vec;   
            //normalized_color = normalized_color/normalized_color.norm();
            //cout <<"delta color, normalized: "<<normalized_color.transpose()<<endl;
            pix_score = normalized_color.dot(reference_color_vec);
            //cout<<"pixel score: "<<pix_score<<endl;
            score += pix_score;

    }
    return score / npixels;
}


// given a column, defined by x_ctr, search over y_ctr-half_width to y_ctr+half_width to get color similarity
double score_col(int x_ctr, int y_ctr, int half_width, Eigen::Vector3d reference_color_vec) {
    Eigen::Vector3d normalized_color;
    double blue, red, green;
    double color_denom;
    int row_start, row_end;
    int col = x_ctr;
    double npixels = half_width * 2 + 1;
    row_start = y_ctr - half_width;
    if (row_start < 0) row_start = 0; // avoid running off the left edge of image
    if (row_start > HEIGHT) row_start = HEIGHT - 1;
    row_end = y_ctr + half_width;
    if (row_end < row_start) row_end = row_start;
    if (row_end > HEIGHT - 1) row_end = HEIGHT - 1;
    double score = 0.0;
    double pix_score = 0.0;

    for (int irow = row_start; irow < row_end; irow++) {
        for (int icolor = 0; icolor < 3; icolor++) {
            normalized_color[icolor] = image.at<cv::Vec3b>(irow, col)[icolor]; //             
        }
            //cout << "x,y, colors: " << icol << ", " << y_ctr << ", " << normalized_color.transpose() << endl;
            normalized_color = normalized_color / (normalized_color.norm() + 1);
            normalized_color -= scene_normalized_avg_color_vec;   
            //normalized_color = normalized_color/normalized_color.norm();
            //cout <<"delta color, normalized: "<<normalized_color.transpose()<<endl;
            pix_score = normalized_color.dot(reference_color_vec);
            //cout<<"pixel score: "<<pix_score<<endl;
            score += pix_score;

    }
    return score / npixels;
}

//score_col_right(xval, y_best, radius_sample, delta_color_vec);
// given a column, defined by x_ctr, search over y_ctr-half_width to y_ctr+half_width to get color similarity
double score_col_right(int x_ctr, int y_ctr, int half_width, Eigen::Vector3d reference_color_vec) {
    Eigen::Vector3d normalized_color;
    double blue, red, green;
    double color_denom;
    int row_start, row_end;
    int col = x_ctr;
    double npixels = half_width * 2 + 1;
    row_start = y_ctr - half_width;
    if (row_start < 0) row_start = 0; // avoid running off the left edge of image
    if (row_start > HEIGHT) row_start = HEIGHT - 1;
    row_end = y_ctr + half_width;
    if (row_end < row_start) row_end = row_start;
    if (row_end > HEIGHT - 1) row_end = HEIGHT - 1;
    double score = 0.0;
    double pix_score = 0.0;

    for (int irow = row_start; irow < row_end; irow++) {
        for (int icolor = 0; icolor < 3; icolor++) {
            normalized_color[icolor] = image_display_right.at<cv::Vec3b>(irow, col)[icolor]; //             
        }
            //cout << "x,y, colors: " << icol << ", " << y_ctr << ", " << normalized_color.transpose() << endl;
            normalized_color = normalized_color / (normalized_color.norm() + 1);
            normalized_color -= scene_normalized_avg_color_vec;   
            //normalized_color = normalized_color/normalized_color.norm();
            //cout <<"delta color, normalized: "<<normalized_color.transpose()<<endl;
            pix_score = normalized_color.dot(reference_color_vec);
            //cout<<"pixel score: "<<pix_score<<endl;
            score += pix_score;

    }
    return score / npixels;
}

void color_fit_markup(double color_tol, double &x_centroid, double &y_centroid) {
    Eigen::Vector3d color_vec, normalized_color_vec, delta_color_vec;
    Eigen::Vector3d nom_ball_normalized_color_vec;
    int nfit=0;
   nom_ball_normalized_color_vec[0] = NOM_BALL_BLUE;
   nom_ball_normalized_color_vec[1] = NOM_BALL_GREEN;
   nom_ball_normalized_color_vec[2] = NOM_BALL_RED;
   nom_ball_normalized_color_vec = nom_ball_normalized_color_vec/nom_ball_normalized_color_vec.norm();    
    //image_markup= cv::imread(g_image_fname); // silly: re-read from disk instead of copy; fix this
    int width = image_markup.cols;
    int height = image_markup.rows; 
    double color_err=0.0;
    x_centroid=0;
    y_centroid=0;    
    for (int y =0; y< height; y++)
        for (int x = 0; x< width; x++)   {
                for (int icolor = 0; icolor < 3; icolor++) {
                        normalized_color_vec[icolor] = image_markup.at<cv::Vec3b>(y,x)[icolor]; //             
                }
            //cout << "x,y, colors: " << x << ", " << y << ", " << normalized_color_vec.transpose() << endl;
            normalized_color_vec = normalized_color_vec / (normalized_color_vec.norm() + 1);  
            color_err = (normalized_color_vec-nom_ball_normalized_color_vec).norm();
            //cout<<"color_err: "<<color_err<<endl;

            if (color_err<color_tol) {
                nfit++;
                x_centroid+=x;
                y_centroid+=y;
                image_markup.at<cv::Vec3b>(y,x)[0] = 0;
                image_markup.at<cv::Vec3b>(y,x)[1] = 0;
                image_markup.at<cv::Vec3b>(y,x)[2] = 255;
            }
        }    
    cout<<" num qualified pixels: "<<nfit<<endl;
    x_centroid/=nfit;
    y_centroid/=nfit;
    cout<<"x_centroid, y_centroid = "<<x_centroid<<", "<<y_centroid<<endl;
    
    
}

Eigen::Vector3d  triangulate(int row,int col_left, int col_right) {
    Eigen::Vector3d xyz_vec;
    double disparity;
    double du_left,du_right;
    du_left = col_left - col_ctr_left;
    du_right = col_right- col_ctr_rt;
    disparity = du_right - du_left; 
    cout<<"disparity = "<<disparity<<endl;
    double x,y,z;
    z = g_baseline*g_focal_length/disparity;
    x = 0.5*((du_right + du_left)/g_focal_length)*z -g_baseline/2; //offset g_baseline to reference w/rt left camera frame
    y = (row/g_focal_length)*z;    
    xyz_vec[0] = x;
    xyz_vec[1] = y;
    xyz_vec[2] = z;  
    return xyz_vec;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "color_interp");
    ros::NodeHandle nh; //standard ros node handle
    ros::ServiceServer service = nh.advertiseService("snapshot_svc", snapshotService);
    //cv::namedWindow(OPENCV_WINDOW);
    double x_centroid,y_centroid;
    vector<string> codes;
    codes.resize(24);
    codes[0]="1_0";  //upper right corner: x,y = 1", 0
    codes[1]="2_0";  //lower right: x,y = 1",1"
    codes[2]="3_0";  //lower left:  x,y = 0", 1"
    codes[3]="4_0";  //upper left:  x,y = 0", 0"
    for (int i=0;i<4;i++)
        g_dist[i]= 0.0;
    for (int i=0;i<24;i+=4) { 
        g_x_world[i+3] = 0.0;//upper left world coords
        g_y_world[i+3] = 0.0;  
        
        g_x_world[i] = 0.0254;//upper right world coords
        g_y_world[i] = 0.0; 

        g_x_world[i+1] = 0.0254;//lower right world coords
        g_y_world[i+1] = 0.0254; 

        g_x_world[i+2] = 0.0;//lower left world coords
        g_y_world[i+2] = 0.0254;         
    }
    
    codes[4]="1_5";
    codes[5]="2_5";
    codes[6]="3_5";
    codes[7]="4_5";
    for (int i=0;i<4;i++)
        g_dist[i+4]= 0.005;    //5mm higher

    codes[8]="1_10";
    codes[9]="2_10";
    codes[10]="3_10";
    codes[11]="4_10";
    for (int i=0;i<4;i++)
        g_dist[i+8]= 0.010;    //10mm higher    
    
    codes[12]="1_15";
    codes[13]="2_15";
    codes[14]="3_15";
    codes[15]="4_15";
    for (int i=0;i<4;i++)
        g_dist[i+12]= 0.015;    //15mm higher    
    
    codes[16]="1_20";
    codes[17]="2_20";
    codes[18]="3_20";
    codes[19]="4_20";
    for (int i=0;i<4;i++)
        g_dist[i+16]= 0.020;    //20mm higher    
    
    codes[20]="1_25";
    codes[21]="2_25";
    codes[22]="3_25";
    codes[23]="4_25";
    for (int i=0;i<4;i++)
        g_dist[i+20]= 0.025;    //25mm higher    
    
  //reverse all distances, s.t. highest value corresponds to z=0, lowest is 0.025m
    for (int i=0;i<24;i++)
        g_dist[i] = 0.025-g_dist[i];
    
    char fname_left[64],fname_right[64],fname_code[16];
    char left_prefix[]= "l"; //left_";
    char right_prefix[] = "r";//"right_";
    char suffix[]=".png";
    //char *strcpy ( char *dest, const char *src );
    //strcpy(prefix,"left_"); //
    //strcpy(suffix,".png"); //char suffix = ".png";

    for(int ipic=0;ipic<24;ipic++)
    {    
    //cout<<"enter code of file name (e.g. 1_0 to 4_25): ";
    //cin>>fname_code;
    strcpy(fname_code,codes[ipic].c_str());//  // from C++ string to old-style C-string
    //char *strcpy ( char *dest, const char *src );
    strcpy(fname_left,left_prefix);
    strcat(fname_left,fname_code); //fname_code);
    strcat(fname_left,suffix);
    

    cout<<"using file name: "<<fname_left<<endl;
    strcpy(fname_right,right_prefix);
    strcat(fname_right,fname_code);
    strcat(fname_right,suffix);   
    
    //strcpy(g_image_fname,fname_left);
    

            
    image_display_left = cv::imread(fname_left); //run this pgm from directory containing named images
    if(! image_display_left.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image "<<fname_left << std::endl ;
        return -1;
    }
    //image_display_left = cv::imread(fname_left); // make a copy for mark-up display
    
    //read corresponding right image
   image_display_right = cv::imread(fname_right); //run this pgm from directory containing named images
    if(! image_display_right.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the right image" << std::endl ;
        return -1;
    }    
    double color_tol=0.2;
    cv::imshow("lcam image ", image_display_left);
    cv::imshow("rcam image ", image_display_right);
    image_markup= image_display_left.clone();
    //image_markup= cv::imread(fname_left); // silly: re-read from disk instead of copy; fix this
    
        //while (ros::ok()) {
    cout<<"image: "<<fname_left<<endl;
        //cv::imshow(fname_left, image_display_left);
        //image_markup= cv::imread(fname_left); // silly: re-read from disk instead of copy; fix this
        cv::imshow("markup left", image_markup);
       
        ros::spinOnce();
        //ros::Duration(0.1).sleep();
        cv::waitKey(30); //need this to allow image display
        //cout<<"enter color tolerance: ";
        //cin>>color_tol;
        cout<<"analyzing left image: ";
        color_fit_markup(color_tol,x_centroid,y_centroid);
        g_x_centroid_left[ipic] = x_centroid;
        g_y_centroid_left[ipic] = y_centroid;        
        for (int i=0;i<30;i++) {
        //cv::imshow(fname_left, image_display_left);        
        cv::imshow("markup left",image_markup);
        cv::waitKey(30);
        }
    //}    
    image_markup= image_display_right.clone();
    //image_markup= cv::imread(fname_left); // silly: re-read from disk instead of copy; fix this
    
    //repeat for right image:
        //while (ros::ok()) {
        //cv::imshow(fname_right, image_display_right);
        //image_markup= cv::imread(fname_right); // silly: re-read from disk instead of copy; fix this
        cv::imshow("markup right", image_markup);
       
        ros::spinOnce();
        //ros::Duration(0.1).sleep();
        cv::waitKey(30); //need this to allow image display
        //cout<<"enter color tolerance: ";
        //cin>>color_tol;
        //cout<<"analyzing right image: ";
        color_fit_markup(color_tol,x_centroid,y_centroid);
        g_x_centroid_right[ipic] = x_centroid;
        g_y_centroid_right[ipic] = y_centroid;         
        for (int i=0;i<30;i++) {
        //cv::imshow(fname_right, image_display_right);        
        cv::imshow("markup right",image_markup);
        cv::waitKey(30);
        }
    }
    cout<<"saving data: "<<endl;
    fstream fs;
    fs.open("calib.data",fstream::out);
    fs<<"world_coords_wrt_cam_frame = ["<<endl;
    for (int i=0;i<NPICS;i++) {
        fs<<g_x_world[i]<<", "<<g_y_world[i]<<","<< g_dist[i]<<endl;
    }
    fs<<"]"<<endl;
    fs<<"uv_vals_left = ["<<endl;
     for (int i=0;i<NPICS;i++) {
        fs<<g_x_centroid_left[i]<<", "<<g_y_centroid_left[i]<<endl;
    }   
    fs<<"]"<<endl; 
    
    fs<<"uv_vals_right = ["<<endl;
     for (int i=0;i<NPICS;i++) {
        fs<<g_x_centroid_right[i]<<", "<<g_y_centroid_right[i]<<endl;
    }   
    fs<<"]"<<endl;    
    fs.close();
    while(ros::ok()) {
        ros::Duration(0.1).sleep();
    }
         return 0;      
}

