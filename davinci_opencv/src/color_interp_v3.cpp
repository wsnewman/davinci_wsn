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
//search params for red ball
int radius = 40;// use 50 for "h" scenes; use 40 for "L" scenes; 
int radius_sample = radius;
int radius_contain = radius*1.5;
int radius_search = radius*3;

//default values: change these, as appropriate
int WIDTH = 640; //set actual values after loading image
int HEIGHT = 480;
int row_ctr = HEIGHT/2;
int col_ctr_left= WIDTH/2;
int col_ctr_rt = WIDTH/2;
const double g_baseline = 0.005; //baseline horizontal separation of cameras, in meters
const double g_focal_length = 1034.0; // focal length, in pixels



static const std::string OPENCV_WINDOW = "Image window";
bool g_trigger = false;

//ugly--make images global:
cv::Mat image, image_display,image_display_right;
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
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle nh; //standard ros node handle
    ros::ServiceServer service = nh.advertiseService("snapshot_svc", snapshotService);
    //cv::namedWindow(OPENCV_WINDOW);
    char fname_left[64],fname_right[64],fname_code[8];
    char left_prefix[]= "l"; //left_";
    char right_prefix[] = "r";//"right_";
    char suffix[]=".png";
    //char *strcpy ( char *dest, const char *src );
    //strcpy(prefix,"left_"); //
    //strcpy(suffix,".png"); //char suffix = ".png";
    
    cout<<"enter code of file name (e.g. 1_h, 1_L, etc): ";
    cin>>fname_code;
    //char *strcpy ( char *dest, const char *src );
    strcpy(fname_left,left_prefix);
    strcat(fname_left,fname_code);
    strcat(fname_left,suffix);
    cout<<"using file name: "<<fname_left<<endl;
    strcpy(fname_right,right_prefix);
    strcat(fname_right,fname_code);
    strcat(fname_right,suffix);    
            
    image = cv::imread(fname_left); //run this pgm from directory containing named images
    if(! image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    image_display = cv::imread(fname_left); // make a copy for mark-up display
    
    //read corresponding right image
   image_display_right = cv::imread(fname_right); //run this pgm from directory containing named images
    if(! image_display_right.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the right image" << std::endl ;
        return -1;
    }    

    cv::imshow("lcam image 1", image_display);
    cv::imshow("rcam image 1", image_display_right);
    Eigen::VectorXd col_rt, col_lft, row_top, row_bot;
    Eigen::Vector3d color_vec, normalized_avg_color_vec, delta_color_vec;
    Eigen::Vector3d pixel_vec;
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
    cout<<"start avg loop"<<endl;
    for (int irow = 0; irow < HEIGHT; irow++)
        for (int j = 0; j < WIDTH; j++) {
            //cout<<"i,j="<<irow<<", "<<j<<endl;
            npixels++;
            // access as (row,col)
            blue += image.at<cv::Vec3b>(irow, j)[0]; //(i,j) is x,y = col, row
            green += image.at<cv::Vec3b>(irow, j)[1];
            red += image.at<cv::Vec3b>(irow, j)[2];
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
    int ystart,yend,xstart,xend;
    ystart = pos.y - radius_sample;
    if (ystart<0) ystart=0;
    yend = pos.y + radius_sample;
    if (yend>HEIGHT-1) yend = HEIGHT-1;
    xstart = pos.x - radius_sample;
    if (xstart<0) xstart=0;
    xend = pos.x + radius_sample;
    if (xend>WIDTH-1) xend = WIDTH-1;
    //compute the color target over sample region about chosen pixel
    for (int var2 = ystart; var2 <= yend; var2++)
        for (int var3 = xstart; var3 <= xend; var3++) {
            npixels++;
            blue += image.at<cv::Vec3b>(var2, var3)[0]; //row, col order = y,x
            green += image.at<cv::Vec3b>(var2, var3)[1];
            red += image.at<cv::Vec3b>(var2, var3)[2];
            //image_display.at<cv::Vec3b>(var2, var3)[0] = 0;
            //image_display.at<cv::Vec3b>(var2, var3)[1] = 0;
            //image_display.at<cv::Vec3b>(var2, var3)[2] = 255;
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
    //delta_color_vec= delta_color_vec/delta_color_vec.norm();
    cout<< "patch delta color vec: "<<delta_color_vec.transpose()<<endl;
    cout<<"dot with self = "<<delta_color_vec.dot(delta_color_vec)<<endl;
    /*
    cout<<"enter 1: ";
    int ans;
    cin>>ans;
    cout<<"test pixels in square region:"<<endl;
      for (int var2 = pos.y - radius_sample; var2 <= pos.y + radius_sample; var2++)
        for (int var3 = pos.x - radius_sample; var3 <= pos.x + radius_sample; var3++) {
            pixel_vec[0] = image.at<cv::Vec3b>(var2, var3)[0]; //row, col order = y,x
            pixel_vec[1] = image.at<cv::Vec3b>(var2, var3)[1];
            pixel_vec[2] = image.at<cv::Vec3b>(var2, var3)[2];
            cout<<"pix colors: "<<pixel_vec.transpose()<<endl;
            pixel_vec/= (pixel_vec.norm()+1);
            pixel_vec-= scene_normalized_avg_color_vec;
            cout<<"row,col = "<<var2<<","<<var3<<"; dpix: "<<pixel_vec.transpose()<<endl;
            cout<<"dot prod: "<<pixel_vec.dot(delta_color_vec)<<endl;
        }
     * */
    //eval a bunch of row and col scores:
    //profile: fill an array of column scores; find the peak cumulative score from col_ctr-radius to col_ctr+radius
    //create second array as integral: first entry is sum(col_min:col_min+2*radius)
    // could use an array from 0 to WIDTH-1
    Eigen::VectorXd col_scores= Eigen::MatrixXd::Zero(WIDTH-1,1);
    Eigen::VectorXd cum_col_scores= Eigen::MatrixXd::Zero(WIDTH-1,1);    
    // next entry is sum(0)
    double col_score;
    xstart = pos.x-radius_search;
    if (xstart<0) xstart=0;
    xend = pos.x + radius_search;
    if (xend>WIDTH-1) xend=WIDTH-1;
    //x val is column; score the color match for a range of columns
    for (int xval = xstart; xval < xend; xval++) {
        col_score = score_col(xval, pos.y, radius_sample, delta_color_vec);
        cout << "xval,  col score: " << xval << ", " << col_score << endl;
        col_scores[xval] = col_score;
    }
    cout<<"line 402"<<endl;
    //now compute cumulative scores over given radius
    xstart = pos.x-radius_search+radius_contain;
    if (xstart<radius_contain) xstart=radius_contain;
    xend = pos.x + radius_search - radius_contain;
    if (xend> WIDTH-1-radius_contain) xend = WIDTH-1-radius_contain;
    for (int xctr = xstart; xctr < xend; xctr++) {
        for (int icol=xctr-radius_contain;icol<xctr+radius_contain;icol++) {
            cum_col_scores[xctr]+=col_scores[icol];
        }
    }
    cout<<"line 413"<<endl;
    int x_best = pos.x-radius_search+radius_contain;
    if (x_best<0) x_best=0;
    if (x_best>WIDTH-1) x_best = WIDTH-1;
    double cum_col_score_best = cum_col_scores[x_best];
    cout<<"xctr, cum score: "<<endl;
    xstart = pos.x-radius_search+radius_contain;
    if (xstart<0) xstart=0;
    xend = pos.x + radius_search - radius_contain;
    if (xend>WIDTH-1) xend=WIDTH-1;
    for (int xctr = xstart; xctr < xend; xctr++) {
        cout<<xctr<<", "<<cum_col_scores[xctr]<<endl;
        if (cum_col_scores[xctr]>cum_col_score_best) {
            cum_col_score_best=cum_col_scores[xctr];
            x_best = xctr;
        }
    }
    cout<<"optimal column, x_best = "<<x_best<<endl;

    // repeat for row scores to find y_best (best column)
    Eigen::VectorXd row_scores= Eigen::MatrixXd::Zero(HEIGHT-1,1);
    Eigen::VectorXd cum_row_scores= Eigen::MatrixXd::Zero(HEIGHT-1,1);    
    // next entry is sum(0)
    double row_score;
    //y val is row; score the color match for a range of rows
    ystart = pos.y-radius_search;
    if (ystart<0) ystart=0;
    yend = pos.y + radius_search;
    if (yend>HEIGHT-1) yend = HEIGHT-1;
    
    for (int yval = ystart; yval < yend; yval++) {
        row_score = score_row(x_best, yval, radius_sample, delta_color_vec);
        if ((yval>-1) && (yval<HEIGHT)) {
            cout << "yval,  row score: " << yval << ", " << row_score << endl;
            row_scores[yval] = row_score;
        }
    }
    
    ystart = pos.y-radius_search+radius_contain;
    if (ystart<radius_contain) ystart=radius_contain;
    yend = pos.y + radius_search - radius_contain;
    if (yend>HEIGHT-1-radius_contain) yend = HEIGHT-1-radius_contain;
    //now compute cumulative scores over given radius
    for (int yctr = ystart; yctr < yend; yctr++) {
        for (int irow=yctr-radius_contain;irow<yctr+radius_contain;irow++) {
            cum_row_scores[yctr]+=row_scores[irow];
        }
    }
    int y_best = pos.y-radius_search+radius_contain;
    if (y_best<0) y_best=0;
    if (y_best>HEIGHT-1) y_best = HEIGHT-1;
    
    double cum_row_score_best = cum_row_scores[y_best];
    cout<<"yctr, cum score: "<<endl;
    
    ystart = pos.y-radius_search+radius_contain;
    if (ystart<0) ystart=0;
    yend = pos.y + radius_search - radius_contain;
    if (yend>HEIGHT-1) yend = HEIGHT-1;
    
    for (int yctr = ystart; yctr < yend; yctr++) {
        cout<<yctr<<", "<<cum_row_scores[yctr]<<endl;
        if (cum_row_scores[yctr]>cum_row_score_best) {
            cum_row_score_best=cum_row_scores[yctr];
            y_best = yctr;
        }
    }
    cout<<"optimal row, y_best = "<<y_best<<endl;
     cout<<"optimal column, x_best = "<<x_best<<endl;   
    //repeat the search over columns, using y_best:
    cout<<"repeat x_best search: "<<endl;
    xstart = x_best-radius_search;
    if (xstart<0) xstart=0;
    xend = x_best + radius_search;
    if (xend>WIDTH-1) xend=WIDTH-1;
   for (int xval = xstart; xval < xend; xval++) {
        col_score = score_col(xval, y_best, radius_sample, delta_color_vec);
        //cout << "xval,  col score: " << xval << ", " << col_score << endl;
        col_scores[xval] = col_score;
    }
    cout<<"line 493"<<endl;
    //now compute cumulative scores over given radius
    xstart = x_best-radius_search+radius_contain;
    if (xstart<radius_contain) xstart=radius_contain;
    xend = x_best + radius_search - radius_contain;
    if (xend>WIDTH-1-radius_contain) xend = WIDTH-1-radius_contain;
    
    for (int xctr = xstart; xctr < xend; xctr++) {
        for (int icol=xctr-radius_contain;icol<xctr+radius_contain;icol++) {
            cum_col_scores[xctr]+=col_scores[icol];
        }
    }
    double x_best_left = x_best-radius_search+radius_contain;
    cum_col_score_best = cum_col_scores[x_best_left];
    //cout<<"xctr, cum score: "<<endl;
    xstart = x_best-radius_search+radius_contain;
    if (xstart<0) xstart=0;
    xend = x_best + radius_search - radius_contain;
    if (xend>WIDTH-1) xend=WIDTH-1;
    for (int xctr = xstart; xctr < xend; xctr++) {
        //cout<<xctr<<", "<<cum_col_scores[xctr]<<endl;
        if (cum_col_scores[xctr]>cum_col_score_best) {
            cum_col_score_best=cum_col_scores[xctr];
            x_best_left = xctr;
        }
    }
    cout<<"optimal column, x_best_left = "<<x_best_left<<endl;    
    
    for (int var2 = y_best - radius_sample; var2 <= y_best + radius_sample; var2++)
        for (int var3 = x_best - radius_sample; var3 <= x_best + radius_sample; var3++) {
            image_display.at<cv::Vec3b>(var2, var3)[0] = 0;
            image_display.at<cv::Vec3b>(var2, var3)[1] = 0;
            image_display.at<cv::Vec3b>(var2, var3)[2] = 255;
        }    
    
    // search for optimal column, given optimal row, in right image:
    //int x_best_right = x_best_left-radius_search;
    //repeat the search over columns, using y_best:
    cout<<"repeat x_best search for right image: "<<endl;
    xstart = x_best_left-2*radius_search;
    if (xstart<0) xstart=0;
    xend = x_best_left + 2*radius_search;
    if (xend>WIDTH-1) xend = WIDTH-1;
    
   for (int xval = xstart; xval < xend; xval++) {
        col_score = score_col_right(xval, y_best, radius_sample, delta_color_vec);
        cout << "xval,  col score: " << xval << ", " << col_score << endl;
        col_scores[xval] = col_score;
    }
    //now compute cumulative scores over given radius
    xstart = x_best_left-2*radius_search;
    if (xstart<radius_contain) xstart=radius_contain;
    xend = x_best_left + 2*radius_search;
    if (xend>WIDTH-1-radius_contain) xend = WIDTH-1-radius_contain;
    cout<<"start loop 515"<<endl;
    for (int xctr = xstart; xctr < xend; xctr++) {
        cum_col_scores[xctr]=0;
        for (int icol=xctr-radius_contain;icol<xctr+radius_contain;icol++) {
            cum_col_scores[xctr]+=col_scores[icol];
        }
    }
    double x_best_right = x_best_left-radius_search+radius_contain;
    cum_col_score_best = cum_col_scores[x_best_right];
    //cout<<"xctr, cum score: "<<endl;
    xstart = x_best_right-2*radius_search+radius_contain;
    if (xstart<0) xstart=0;
    xend = x_best_right + 2*radius_search - radius_contain;
    if (xend>WIDTH-1) xend = WIDTH-1;
    for (int xctr = xstart; xctr < xend; xctr++) {
        //cout<<xctr<<", "<<cum_col_scores[xctr]<<endl;
        if (cum_col_scores[xctr]>cum_col_score_best) {
            cum_col_score_best=cum_col_scores[xctr];
            x_best_right = xctr;
        }
    }
    cout<<"optimal column, x_best_right = "<<x_best_right<<endl;    
    cout<<"optimal column, x_best_left = "<<x_best_left<<endl; 
    cout<<"optimal row, y_best = "<<y_best<<endl;
    
    for (int var2 = y_best - radius_sample; var2 <= y_best + radius_sample; var2++)
        for (int var3 = x_best_right - radius_sample; var3 <= x_best_right + radius_sample; var3++) {
            image_display_right.at<cv::Vec3b>(var2, var3)[0] = 0;
            image_display_right.at<cv::Vec3b>(var2, var3)[1] = 0;
            image_display_right.at<cv::Vec3b>(var2, var3)[2] = 255;
        }        
    
    Eigen::Vector3d  xyz_vec;
    xyz_vec = triangulate(y_best,x_best_left,x_best_right);
    cout<<"triangulated x,y,z: "<<xyz_vec.transpose()<<endl;
    while (ros::ok()) {
        cv::imshow("lcam image 1", image_display);
        cv::imshow("rcam image 1", image_display_right);
        ros::spinOnce();
        //ros::Duration(0.1).sleep();
        cv::waitKey(30); //need this to allow image display
    }
    //ImageConverter ic;
    //ros::spin();
    return 0;
}

