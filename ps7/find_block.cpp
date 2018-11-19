//get images from topic "simple_camera/image_raw"; remap, as desired;
//search for red pixels;
// convert (sufficiently) red pixels to white, all other pixels black
// compute centroid of red pixels and display as a blue square
// publish result of processed image on topic "/image_converter/output_video"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <xform_utils/xform_utils.h>
#include <cmath>

static const std::string OPENCV_WINDOW = "Open-CV display window";
using namespace std;

double g_scale = 0.002967; //ORIGINAL
//double g_scale = 0.0030197051;
double g_central_X = 0.543;
double g_central_Y = 0.321;
int g_central_I = 319;
int g_central_J = 239;
double g_theta = 0.205048;

// Temp stuff used to replace what theta is meant to be, used in getting value for the transofrm matrix
//double quatX = 1.021206909;
double quatX = 0.9800639326;
//double quatY = -.2070351423;
double quatY = -0.1986823795;
int g_redratio; //threshold to decide if a pixel qualifies as dominantly "red"

const double BLOCK_HEIGHT=0.035; // hard-coded top surface of block relative to world frame

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher block_pose_publisher_; // = n.advertise<std_msgs::Float64>("topic1", 1);
    geometry_msgs::PoseStamped block_pose_;
    XformUtils xformUtils;

public:

    ImageConverter(ros::NodeHandle &nodehandle)
    : it_(nh_) {
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("simple_camera/image_raw", 1,
                &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        block_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("block_pose", 1, true); 
        block_pose_.header.frame_id = "world"; //specify the  block pose in world coords
        block_pose_.pose.position.z = BLOCK_HEIGHT;
        block_pose_.pose.position.x = 0.5; //not true, but legal
        block_pose_.pose.position.y = 0.0; //not true, but legal
        
        // need camera info to fill in x,y,and orientation x,y,z,w
        //geometry_msgs::Quaternion quat_est
        //quat_est = xformUtils.convertPlanarPsi2Quaternion(yaw_est);
        block_pose_.pose.orientation = xformUtils.convertPlanarPsi2Quaternion(0); //not true, but legal
        
        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    //image comes in as a ROS message, but gets converted to an OpenCV type
    void imageCb(const sensor_msgs::ImageConstPtr& msg); 
    
}; //end of class definition

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr cv_ptr; //OpenCV data type
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // look for red pixels; turn all other pixels black, and turn red pixels white
        int npix = 0; //count the red pixels
        int isum = 0; //accumulate the column values of red pixels
        int jsum = 0; //accumulate the row values of red pixels
        int redval, blueval, greenval, testval;
        int xMin_X = 641, xMin_Y = 481, yMin_X = 641, yMin_Y = 481, xMax_X = -1, xMax_Y = -1;
        double distance = 0.0, distance_x = 0.0;
        double angle = 0.0;
        cv::Vec3b rgbpix; // OpenCV representation of an RGB pixel
        //comb through all pixels (j,i)= (row,col)
        for (int i = 0; i < cv_ptr->image.cols; i++) {
            for (int j = 0; j < cv_ptr->image.rows; j++) {
                rgbpix = cv_ptr->image.at<cv::Vec3b>(j, i); //extract an RGB pixel
                //examine intensity of R, G and B components (0 to 255)
                redval = rgbpix[2] + 1; //add 1, to avoid divide by zero
                blueval = rgbpix[0] + 1;
                greenval = rgbpix[1] + 1;
                //look for red values that are large compared to blue+green
                testval = redval / (blueval + greenval);
                //if red (enough), paint this white:
                if (testval > g_redratio) {
                    cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 255;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 255;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 255;
                    npix++; //note that found another red pixel
                    isum += i; //accumulate row and col index vals
                    jsum += j;
                    
                    // Check for edge for use in orientation:
                    if(i < xMin_X){
                    	xMin_X = i;
                    	xMin_Y = j;
                    }
                    if(j < yMin_Y){
                    	yMin_X = i;
                    	yMin_Y = j;
                    }
                    if(i > xMax_X){
                    	xMax_X = i;
                    	xMax_Y = j;
                    }                    
                } else { //else paint it black
                    cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 0;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 0;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 0;
                }
            }
        }
        //used to check if side is a small or big side
        distance = sqrt((xMin_X - yMin_X)*(xMin_X - yMin_X) + (xMin_Y - yMin_Y)*(xMin_Y - yMin_Y));
        //used to check if needed to rotate 90 deg
        distance_x = sqrt((xMax_X - xMin_X)*(xMax_X - xMin_X) + (xMax_Y - xMin_Y)*(xMax_Y - xMin_Y));
        ROS_INFO("Before angle calc");
        // Add a null condition for the division
        if((yMin_Y - xMin_Y) == 0){
        	angle = 0;
        }
        else{
        	angle = atan((yMin_X - xMin_X)/(yMin_Y - xMin_Y));
        }
        ROS_INFO("After angle calc");
        // CALCULATE ORIENTATION HERE
        // if distance between xmin and ymin are small, rotate the orientation 90deg
        if((distance != 0 && distance < 20) || (distance == 0 && distance_x > 20)){
	        angle += M_PI/2;
        }
        
        //cout << "npix: " << npix << endl;
        //paint in a blue square at the centroid:
        int half_box = 5; // choose size of box to paint
        int i_centroid, j_centroid;
        double x_centroid, y_centroid;
        if (npix > 0) {
            i_centroid = isum / npix; // average value of u component of red pixels
            j_centroid = jsum / npix; // avg v component
            x_centroid = ((double) isum)/((double) npix); //floating-pt version
            y_centroid = ((double) jsum)/((double) npix);
            ROS_INFO("u_avg: %f; v_avg: %f",x_centroid,y_centroid);
            //cout << "i_avg: " << i_centroid << endl; //i,j centroid of red pixels
            //cout << "j_avg: " << j_centroid << endl;
            for (int i_box = i_centroid - half_box; i_box <= i_centroid + half_box; i_box++) {
                for (int j_box = j_centroid - half_box; j_box <= j_centroid + half_box; j_box++) {
                    //make sure indices fit within the image 
                    if ((i_box >= 0)&&(j_box >= 0)&&(i_box < cv_ptr->image.cols)&&(j_box < cv_ptr->image.rows)) {
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[0] = 255; //(255,0,0) is pure blue
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[1] = 0;
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[2] = 0;
                    }
                }
            }

        }
        // Update GUI Window; this will display processed images on the open-cv viewer.
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3); //need waitKey call to update OpenCV image window

        // Also, publish the processed image as a ROS message on a ROS topic
        // can view this stream in ROS with: 
        //rosrun image_view image_view image:=/image_converter/output_video
        image_pub_.publish(cv_ptr->toImageMsg());
        
        // Values for U and V, used for finding the rest
        double u = (i_centroid - g_central_I) * g_scale;
        double v = (j_centroid - g_central_J) * g_scale	;
        
        // OG way, used for the X value as the Y potion of this was not getting the correct value as expected
        // Value of theta found by using a system of equations to solve, as described in the writeup
        block_pose_.pose.position.x = cos(g_theta) * u - sin(g_theta) * v + g_central_X; //not true, but legal
        //block_pose_.pose.position.y = sin(g_theta) * u - cos(g_theta) * v + g_central_Y; //not true, but legal
        
        // New Way, used for y position as the X value for this was inaccurate, like above
        //block_pose_.pose.position.x = quatX * u - quatY * v + g_central_X; //not true, but legal
        block_pose_.pose.position.y = quatY * u - quatX * v + g_central_Y; // Values obtained using the same J value to fill out the matrix
        
        //block_pose_.pose.position.y = block_pose_.pose.position.y - 2 * (g_central_Y - block_pose_.pose.position.y);
        ROS_INFO("x_r: %f; y_r: %f",block_pose_.pose.position.x,block_pose_.pose.position.y);
        double theta=g_theta; // was 0 
        
        // Uncomment line, but might break code sometimes. Includes orientation of block and orientation of the camera in one to make the tool pose rotate tothe new goalpos
        theta = g_theta + angle;
        
        
        // need camera info to fill in x,y,and orientation x,y,z,w
        //geometry_msgs::Quaternion quat_est
        //quat_est = xformUtils.convertPlanarPsi2Quaternion(yaw_est);
        block_pose_.pose.orientation = xformUtils.convertPlanarPsi2Quaternion(angle); //not true, but legal
        //block_pose_.pose.orientation.z = angle;
        block_pose_publisher_.publish(block_pose_);
    }

int main(int argc, char** argv) {
    ros::init(argc, argv, "red_pixel_finder");
    ros::NodeHandle n; //        
    ImageConverter ic(n); // instantiate object of class ImageConverter
    //cout << "enter red ratio threshold: (e.g. 10) ";
    //cin >> g_redratio;
    g_redratio= 10; //choose a threshold to define what is "red" enough
    ros::Duration timer(0.1);
    double x, y, z;
    while (ros::ok()) {
        ros::spinOnce();
        timer.sleep();
    }
    return 0;
}
