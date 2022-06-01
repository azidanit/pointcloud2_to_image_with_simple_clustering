#include <ros/ros.h>
#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>

#include <iostream>
#include <vector>

using namespace std;

cv::Mat image;
cv::Mat drawing;

typedef pcl::PointCloud<pcl::PointXY> PointCloud;
visualization_msgs::MarkerArray object_cube;

ros::Publisher objectsArray_pub;

inline double calculateResultant(double a, double b){
    return sqrt(pow(a,2) + pow(b,2));
}

inline double calculateDistance(double x0, double y0, double x1, double y1){
    return calculateResultant(x1-x0, y1-y0);
}

void pointcloud2Callbak(PointCloud* msg)
{
    image = cv::Mat::zeros(500,500, CV_8U);

//    printf("GET Lidar\n");
//    printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    BOOST_FOREACH (const pcl::PointXY& pt, msg->points){
       printf ("\t(%f, %f)\n", pt.x, pt.y);
        if(pt.x >= -5 && pt.x <= 5 && pt.y >=-5 && pt.y <=5){
            int y = 500 - ((pt.x * 50) + 250);
            int x = 500 - ((pt.y * 50) + 250);
            image.at<uchar>(cv::Point (x,y)) = 255;
        }
    }

    cv::Mat element_d = cv::getStructuringElement( cv::MORPH_RECT,
                                                   cv::Size( 5, 5 ));
    cv::Mat element_e = cv::getStructuringElement( cv::MORPH_RECT,
                                                   cv::Size( 7, 7 ));

    cv::dilate( image, image, element_d );
    // cv::dilate( image, image, element_d );
//    cv::erode( image, image, element_e );

    cv::dilate( image, image, element_d );

    cv::Mat canny_output;
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    cv::RNG rng(12345);

    /// Find contours20
    cv::findContours( image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    /// Draw contours
    cv::cvtColor(image, drawing, cv::COLOR_GRAY2BGR);
    cv::Rect boundRect;
    cv::RotatedRect rotateRect;

    visualization_msgs::Marker obj;

    obj.header.frame_id = "laser_link";
    obj.header.stamp = ros::Time::now();
    obj.ns = "obs";
    obj.type = visualization_msgs::Marker::CUBE;
    obj.action = visualization_msgs::Marker::DELETEALL;
    object_cube.markers.clear();
    object_cube.markers.push_back(obj);
    objectsArray_pub.publish(object_cube);

    obj.action = visualization_msgs::Marker::ADD;

    obj.pose.orientation.x = 0.0;
    obj.pose.orientation.y = 0.0;
    obj.pose.orientation.z = 0.0;
    obj.pose.orientation.w = 1.0;
    obj.color.a = .6; // Don't forget to set the alpha!
    obj.color.r = 0.0;
    obj.color.g = 0.5;
    obj.color.b = 0.7;

    int x0,y0,x1,y1;

    for( int i = 0; i< contours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar( 255,0,0);
        boundRect = cv::boundingRect( contours[i] );
        rotateRect = cv::minAreaRect(contours[i]);
//        minBoundRect = cv::minAreaRect( contours[i] );
//        cv::rectangle( drawing, boundRect.tl(), boundRect.br(), color, 2 );

        color = cv::Scalar( 0,255,0);
        cv::Point2f rect_points[4];
        rotateRect.points( rect_points );
        for ( int j = 0; j < 4; j++ )
        {
            line( drawing, rect_points[j], rect_points[(j+1)%4], color );
        }

//        std::cout << rect_points[0] << "\t" << rect_points[1] << "\t" << rect_points[2] << "\t" << rect_points[3] << std::endl;

//        x0 = (500 - boundRect.tl().y - 250) * 20 / 250 ;
//        y0 = (boundRect.tl().x - 250) * 20 / 250 * -1;
//        x1 = (500 - boundRect.br().y - 250) * 20 / 250;
//        y1 = (boundRect.br().x - 250) * 20 / 250 * -1;

//        [170, 133]	[170, 124]	[181, 124]	[181, 133]

        x0 = (500 - rect_points[0].y - 250) * 5 / 250 ;
        y0 = (rect_points[0].x - 250) * 5 / 250 * -1;
        x1 = (500 - rect_points[2].y - 250) * 5 / 250;
        y1 = (rect_points[2].x - 250) * 5 / 250 * -1;

//        std::cout << rotateRect.angle << "\n";

//        obj.id = i;
//        obj.scale.x = fabs(x1-x0);
//        obj.scale.y = fabs(y1-y0);
//        obj.scale.z = 1.0;

        obj.id = i;
        obj.scale.x = (calculateDistance(rect_points[0].x, rect_points[0].y, rect_points[1].x, rect_points[1].y)) * 5 / 250;
        obj.scale.y = (calculateDistance(rect_points[0].x, rect_points[0].y, rect_points[3].x, rect_points[3].y)) * 5 / 250;
        obj.scale.z = 1.0;

        tf::Quaternion q;
        q.setRPY( 0,0,-rotateRect.angle * M_PI / 180);
//    person_marker.points
        obj.pose.orientation.x = q.x();
        obj.pose.orientation.y = q.y();
        obj.pose.orientation.z = q.z();
        obj.pose.orientation.w = q.w();

//        obj.pose.position.x = (x0+x1)/2.0;
//        obj.pose.position.y = (y0+y1)/2.0;
//        obj.pose.position.z = 0.0;

        obj.pose.position.x = (x0+x1) / 2.0 ;
        obj.pose.position.y = (y0+y1) / 2.0 ;
        obj.pose.position.z = 0.0;

        object_cube.markers.push_back(obj);

    }

    objectsArray_pub.publish(object_cube);

}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXY> cloud;
    pcl::fromROSMsg (*input, cloud);
    std::cout << "get pcl2msg\n";
    pointcloud2Callbak(&cloud);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_ac222");

    std::cout << "PCL ACCES STARTED\n";
    image = cv::Mat::zeros(500,500, CV_8U);
    drawing = cv::Mat::zeros(500,500, CV_8U);


    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe ("/lslidar_point_cloud", 1, cloud_cb);
    objectsArray_pub = nh.advertise<visualization_msgs::MarkerArray>("/lidar/objects", 1);

    ros::Rate r(50);

    while(ros::ok()){
        cv::imshow("rawsss", image);
        cv::imshow("out", drawing);

        cv::waitKey(1);
        ros::spinOnce();

        r.sleep();
    }


    return 0;
}