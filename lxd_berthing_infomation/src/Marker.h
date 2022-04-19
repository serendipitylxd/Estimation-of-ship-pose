//
// Created by luxiaodong on 2022/1/20.
//

#ifndef LXD_BERTHING_INFOMATION_MARKER_H
#define LXD_BERTHING_INFOMATION_MARKER_H

//
#include <visualization_msgs/Marker.h>
#include<iostream>
#include<pcl/point_types.h>
#include "Other_berthing_infomation_calculation.h"
#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
//
using namespace std;

//
visualization_msgs::Marker line_strip_1;

visualization_msgs::Marker line_strip_2;

visualization_msgs::Marker line_strip_3;

visualization_msgs::Marker line_strip_4;

visualization_msgs::Marker line_strip_5;


visualization_msgs::Marker sphere_stern;

visualization_msgs::Marker sphere_amidship;

visualization_msgs::Marker sphere_bow;

visualization_msgs::Marker text_view_facing_1;

visualization_msgs::Marker text_view_facing_2;

visualization_msgs::Marker text_view_facing_3;

visualization_msgs::Marker text_view_facing_4;

visualization_msgs::Marker text_view_facing_5;

visualization_msgs::Marker text_view_facing_6;



class Draw_marker{

public:


    void draw_line(Other_berthing_infomation_calculation otherBerthingInfomationCalculation){

        visualization_msgs::Marker Line_strip_1;
        line_strip_1 = Line_strip_1;

        //Visualization of bow distance
        line_strip_1.header.frame_id = "lidar";
        line_strip_1.header.stamp = ros::Time::now();
        line_strip_1.ns = "target_bow_line";//namespace
        line_strip_1.action = visualization_msgs::Marker::ADD;
        line_strip_1.pose.orientation.w = 1.0;
        line_strip_1.id = 0; //unique id, useful when multiple markers exist.
        line_strip_1.type = visualization_msgs::Marker::LINE_STRIP; //marker type
        line_strip_1.scale.x = 1; //width of the line
        line_strip_1.color.r = 1.0; line_strip_1.color.g = 0.0; line_strip_1.color.b = 1.0;//color of the line
        line_strip_1.color.a = 1.0; //Opacity, set 0 to full transparency

        geometry_msgs:: Point p1[1];
        //Coordinates of the end points of a line
        p1[0].x = otherBerthingInfomationCalculation.P_bow_x;
        p1[0].y = otherBerthingInfomationCalculation.P_bow_y;
        p1[0].z = otherBerthingInfomationCalculation.P_bow_z;
        p1[1].x = otherBerthingInfomationCalculation.P_bow_x;
        p1[1].y = 0;
        p1[1].z = otherBerthingInfomationCalculation.P_bow_z;

        //The LINE_STRIP type joins only two adjacent points in line_strip.points, such as 0 and 1,1 and 2,2 and 3

        line_strip_1.points.push_back(p1[0]);
        line_strip_1.points.push_back(p1[1]);//




        //Visualization of amidship distance
        visualization_msgs::Marker Line_strip_2;
        line_strip_2 = Line_strip_2;

        line_strip_2.header.frame_id = "lidar";
        line_strip_2.header.stamp = ros::Time::now();
        line_strip_2.ns = "target_amidship_line_1";//namespace
        line_strip_2.action = visualization_msgs::Marker::ADD;
        line_strip_2.pose.orientation.w = 1.0;
        line_strip_2.id = 1; //unique id, useful when multiple markers exist.
        line_strip_2.type = visualization_msgs::Marker::LINE_STRIP; //marker type
        line_strip_2.scale.x = 1; //width of the line
        line_strip_2.color.r = 1.0; line_strip_2.color.g = 215.0/255.0; line_strip_2.color.b = 0.0;//color of the line
        line_strip_2.color.a = 1.0;
        geometry_msgs:: Point p2[1];

        p2[0].x = otherBerthingInfomationCalculation.P_amidship_x;
        p2[0].y = otherBerthingInfomationCalculation.P_amidship_y;
        p2[0].z = otherBerthingInfomationCalculation.P_amidship_z;
        p2[1].x = otherBerthingInfomationCalculation.P_amidship_x;
        p2[1].y = 0;
        p2[1].z = otherBerthingInfomationCalculation.P_amidship_z;

        line_strip_2.points.push_back(p2[0]);
        line_strip_2.points.push_back(p2[1]);


        //Visualization of stern distance
        visualization_msgs::Marker Line_strip_3;
        line_strip_3 = Line_strip_3;

        line_strip_3.header.frame_id = "lidar";
        line_strip_3.header.stamp = ros::Time::now();
        line_strip_3.ns = "target_stern_line_2";//namespace
        line_strip_3.action = visualization_msgs::Marker::ADD;
        line_strip_3.pose.orientation.w = 1.0;
        line_strip_3.id = 2; //unique id, useful when multiple markers exist.
        line_strip_3.type = visualization_msgs::Marker::LINE_STRIP; //marker type
        line_strip_3.scale.x = 1; //width of the line
        line_strip_3.color.r = 0.0; line_strip_3.color.g = 1.0; line_strip_3.color.b = 0.0;//color of the line
        line_strip_3.color.a = 1.0;
        geometry_msgs:: Point p3[1];

        p3[0].x = otherBerthingInfomationCalculation.P_stern_x;
        p3[0].y = otherBerthingInfomationCalculation.P_stern_y;
        p3[0].z = otherBerthingInfomationCalculation.P_stern_z;
        p3[1].x = otherBerthingInfomationCalculation.P_stern_x;
        p3[1].y = 0;
        p3[1].z = otherBerthingInfomationCalculation.P_stern_z;


        line_strip_3.points.push_back(p3[0]);
        line_strip_3.points.push_back(p3[1]);



        //Distance Boffset visualization
        visualization_msgs::Marker Line_strip_4;
        line_strip_4 = Line_strip_4;

        line_strip_4.header.frame_id = "lidar";
        line_strip_4.header.stamp = ros::Time::now();
        line_strip_4.ns = "target_offset";//namespace
        line_strip_4.action = visualization_msgs::Marker::ADD;
        line_strip_4.pose.orientation.w = 1.0;
        line_strip_4.id = 3; //unique id, useful when multiple markers exist.
        line_strip_4.type = visualization_msgs::Marker::LINE_STRIP; //marker type
        line_strip_4.scale.x = 1; //width of the line
        line_strip_4.color.r = 0.0; line_strip_4.color.g = 1.0; line_strip_4.color.b = 1.0;//color of the line
        line_strip_4.color.a = 1.0; // Opacity, set to 0 for full transparency
        geometry_msgs:: Point p4[1];

        p4[0].x = otherBerthingInfomationCalculation.P_amidship_x;
        p4[0].y = 0;
        p4[0].z = otherBerthingInfomationCalculation.P_amidship_z;
        p4[1].x = 0;
        p4[1].y = 0;
        p4[1].z = otherBerthingInfomationCalculation.P_amidship_z;



        line_strip_4.points.push_back(p4[0]);
        line_strip_4.points.push_back(p4[1]);

        visualization_msgs::Marker Line_strip_5;
        line_strip_5 = Line_strip_5;

        //Visualization of ship's nesrest distance from shore
        line_strip_5.header.frame_id = "lidar";
        line_strip_5.header.stamp = ros::Time::now();
        line_strip_5.ns = "target_d_nesrest";//namespace
        line_strip_5.action = visualization_msgs::Marker::ADD;
        line_strip_5.pose.orientation.w = 1.0;
        line_strip_5.id = 8; //unique id, useful when multiple markers exist.
        line_strip_5.type = visualization_msgs::Marker::LINE_STRIP; //marker type
        line_strip_5.scale.x = 1; //width of the line
        line_strip_5.color.r = 139.0/255.0; line_strip_5.color.g = 0.0; line_strip_5.color.b = 139.0/255.0;//color of the line
        line_strip_5.color.a = 1.0;

        geometry_msgs:: Point p[2];

        p[0].x = otherBerthingInfomationCalculation.P_nearest_x; p[0].y = otherBerthingInfomationCalculation.P_nearest_y; p[0].z = otherBerthingInfomationCalculation.P_nearest_z;
        p[1].x = otherBerthingInfomationCalculation.P_nearest_x; p[1].y = 0; p[1].z = otherBerthingInfomationCalculation.P_nearest_z;

        line_strip_5.points.push_back(p[0]);
        line_strip_5.points.push_back(p[1]);//

        //    marker_pub.publish(line_strip);

    }

    void draw_sphere(Other_berthing_infomation_calculation otherBerthingInfomationCalculation){

        //Visualization of stern key points
        visualization_msgs::Marker Sphere_stern;
        sphere_stern = Sphere_stern;

        sphere_stern.header.frame_id = "lidar";
        sphere_stern.header.stamp = ros::Time::now();
        sphere_stern.ns = "target_stern_key_point";//namespace
        sphere_stern.action = visualization_msgs::Marker::ADD;
        sphere_stern.pose.orientation.w = 1.0;
        sphere_stern.id = 4; //unique id, useful when multiple markers exist.
        sphere_stern.type = visualization_msgs::Marker::SPHERE; //marker type
        sphere_stern.pose.position.x = otherBerthingInfomationCalculation.P_stern_x;
        sphere_stern.pose.position.y = otherBerthingInfomationCalculation.P_stern_y;
        sphere_stern.pose.position.z = otherBerthingInfomationCalculation.P_stern_z;
        sphere_stern.scale.x = 5;
        sphere_stern.scale.y = 5;
        sphere_stern.scale.z = 5;
        sphere_stern.color.a = 1.0; // Don't forget to set the alpha!,the opacity
        sphere_stern.color.r = 1.0;
        sphere_stern.color.g = 0.0;
        sphere_stern.color.b = 0.0;

        //Visualization of amidship key points
        visualization_msgs::Marker Sphere_amidship;
        sphere_amidship = Sphere_amidship;

        sphere_amidship.header.frame_id = "lidar";
        sphere_amidship.header.stamp = ros::Time::now();
        sphere_amidship.ns = "target_amidship_key_point";//namespace
        sphere_amidship.action = visualization_msgs::Marker::ADD;
        sphere_amidship.pose.orientation.w = 1.0;
        sphere_amidship.id = 5; //unique id, useful when multiple markers exist.
        sphere_amidship.type = visualization_msgs::Marker::SPHERE; //marker type
        sphere_amidship.pose.position.x = otherBerthingInfomationCalculation.P_amidship_x;
        sphere_amidship.pose.position.y = otherBerthingInfomationCalculation.P_amidship_y;
        sphere_amidship.pose.position.z = otherBerthingInfomationCalculation.P_amidship_z;
        sphere_amidship.scale.x = 5;
        sphere_amidship.scale.y = 5;
        sphere_amidship.scale.z = 5;
        sphere_amidship.color.a = 1.0; // Don't forget to set the alpha!,the opacity
        sphere_amidship.color.r = 1.0;
        sphere_amidship.color.g = 0.0;
        sphere_amidship.color.b = 0.0;

        //Visualization of bow key points
        visualization_msgs::Marker Sphere_bow;
        sphere_bow = Sphere_bow;

        sphere_bow.header.frame_id = "lidar";
        sphere_bow.header.stamp = ros::Time::now();
        sphere_bow.ns = "target_bow_key_point";//namespace
        sphere_bow.action = visualization_msgs::Marker::ADD;
        sphere_bow.pose.orientation.w = 1.0;
        sphere_bow.id = 6; //unique id, useful when multiple markers exist.
        sphere_bow.type = visualization_msgs::Marker::SPHERE; //marker type
        sphere_bow.pose.position.x = otherBerthingInfomationCalculation.P_bow_x;
        sphere_bow.pose.position.y = otherBerthingInfomationCalculation.P_bow_y;
        sphere_bow.pose.position.z = otherBerthingInfomationCalculation.P_bow_z;
        sphere_bow.scale.x = 5;
        sphere_bow.scale.y = 5;
        sphere_bow.scale.z = 5;
        sphere_bow.color.a = 1.0; // The opacity
        sphere_bow.color.r = 1.0;
        sphere_bow.color.g = 0.0;
        sphere_bow.color.b = 0.0;


    }

    void draw_text(Other_berthing_infomation_calculation otherBerthingInfomationCalculation,double headingAngle){

        visualization_msgs::Marker Text_view_facing_1;
        text_view_facing_1 = Text_view_facing_1;

        //Text display Settings
        text_view_facing_1.header.frame_id = "lidar";
        text_view_facing_1.header.stamp = ros::Time::now();
        text_view_facing_1.ns = "target_text1";//namespace
        text_view_facing_1.action = visualization_msgs::Marker::ADD;
        text_view_facing_1.pose.orientation.w = 1.0;
        text_view_facing_1.id = 9; //unique id, useful when multiple markers exist.
        text_view_facing_1.type = visualization_msgs::Marker::TEXT_VIEW_FACING; //marker type
        text_view_facing_1.pose.position.x = 0;
        text_view_facing_1.pose.position.y = 0;
        text_view_facing_1.pose.position.z = 100;

        text_view_facing_1.scale.z = 20;
        text_view_facing_1.color.a = 1.0; // The opacity
        text_view_facing_1.color.r = 0.0;
        text_view_facing_1.color.g = 1.0;
        text_view_facing_1.color.b = 1.0;

        ostringstream str;
        str << "d_boffset: " << otherBerthingInfomationCalculation.P_amidship_x << " m," << " v_xamidship: " << otherBerthingInfomationCalculation.v_amidship_x << " m/s";

        text_view_facing_1.text=str.str();

        visualization_msgs::Marker Text_view_facing_2;
        text_view_facing_2 = Text_view_facing_2;

        //Text display Settings
        text_view_facing_2.header.frame_id = "lidar";
        text_view_facing_2.header.stamp = ros::Time::now();
        text_view_facing_2.ns = "target_text2";//namespace
        text_view_facing_2.action = visualization_msgs::Marker::ADD;
        text_view_facing_2.pose.orientation.w = 1.0;
        text_view_facing_2.id = 10; //unique id, useful when multiple markers exist.
        text_view_facing_2.type = visualization_msgs::Marker::TEXT_VIEW_FACING; //marker type
        text_view_facing_2.pose.position.x = 0;
        text_view_facing_2.pose.position.y = 0;
        text_view_facing_2.pose.position.z = 160;

        text_view_facing_2.scale.z = 20;
        text_view_facing_2.color.a = 1.0; // The opacity
        text_view_facing_2.color.r = 1.0;
        text_view_facing_2.color.g = 0.0;
        text_view_facing_2.color.b = 1.0;

        ostringstream str2;
        str2 << "d_bow: " << otherBerthingInfomationCalculation.P_bow_y << " m," << " v_bow: " << otherBerthingInfomationCalculation.v_bow << " m/s,"<< " v_ybow: " << otherBerthingInfomationCalculation.v_bow_y << " m/s";

        text_view_facing_2.text=str2.str();

        visualization_msgs::Marker Text_view_facing_3;
        text_view_facing_3 = Text_view_facing_3;

        //Text display Settings
        text_view_facing_3.header.frame_id = "lidar";
        text_view_facing_3.header.stamp = ros::Time::now();
        text_view_facing_3.ns = "target_text3";//namespace
        text_view_facing_3.action = visualization_msgs::Marker::ADD;
        text_view_facing_3.pose.orientation.w = 1.0;
        text_view_facing_3.id = 11; //unique id, useful when multiple markers exist.
        text_view_facing_3.type = visualization_msgs::Marker::TEXT_VIEW_FACING; //marker type
        text_view_facing_3.pose.position.x = 0;
        text_view_facing_3.pose.position.y = 0;
        text_view_facing_3.pose.position.z = 140;

        text_view_facing_3.scale.z = 20;
        text_view_facing_3.color.a = 1.0; // The opacity
        text_view_facing_3.color.r = 1.0;
        text_view_facing_3.color.g = 215.0/255.0;
        text_view_facing_3.color.b = 0.0;

        ostringstream str3;
        str3 << "d_amidship: " << otherBerthingInfomationCalculation.P_amidship_y << " m,"<< "v_amidship: " << otherBerthingInfomationCalculation.v_amidship << " m/s," << " v_yamidship: " << otherBerthingInfomationCalculation.v_amidship_y << " m/s";

        text_view_facing_3.text=str3.str();

        visualization_msgs::Marker Text_view_facing_4;
        text_view_facing_4 = Text_view_facing_4;

        //Text display Settings
        text_view_facing_4.header.frame_id = "lidar";
        text_view_facing_4.header.stamp = ros::Time::now();
        text_view_facing_4.ns = "target_text4";//namespace
        text_view_facing_4.action = visualization_msgs::Marker::ADD;
        text_view_facing_4.pose.orientation.w = 1.0;
        text_view_facing_4.id = 12; //unique id, useful when multiple markers exist.
        text_view_facing_4.type = visualization_msgs::Marker::TEXT_VIEW_FACING; //marker type
        text_view_facing_4.pose.position.x = 0;
        text_view_facing_4.pose.position.y = 0;
        text_view_facing_4.pose.position.z = 120;

        text_view_facing_4.scale.z = 20;
        text_view_facing_4.color.a = 1.0; // The opacity
        text_view_facing_4.color.r = 0.0;
        text_view_facing_4.color.g = 1.0;
        text_view_facing_4.color.b = 0.0;

        ostringstream str4;
        str4 << "d_stern: " << otherBerthingInfomationCalculation.P_stern_y << " m,"<< " v_stern: " << otherBerthingInfomationCalculation.v_stern << " m/s," << " v_ystern: " << otherBerthingInfomationCalculation.v_stern_y << " m/s";

        text_view_facing_4.text=str4.str();

        visualization_msgs::Marker Text_view_facing_5;
        text_view_facing_5 = Text_view_facing_5;

        //Text display Settings
        text_view_facing_5.header.frame_id = "lidar";
        text_view_facing_5.header.stamp = ros::Time::now();
        text_view_facing_5.ns = "target_text5";//namespace
        text_view_facing_5.action = visualization_msgs::Marker::ADD;
        text_view_facing_5.pose.orientation.w = 1.0;
        text_view_facing_5.id = 13; //unique id, useful when multiple markers exist.
        text_view_facing_5.type = visualization_msgs::Marker::TEXT_VIEW_FACING; //marker type
        text_view_facing_5.pose.position.x = 0;
        text_view_facing_5.pose.position.y = 0;
        text_view_facing_5.pose.position.z = 200;

        text_view_facing_5.scale.z = 20;
        text_view_facing_5.color.a = 1.0; // The opacity
        text_view_facing_5.color.r = 255.0;
        text_view_facing_5.color.g = 255.0;
        text_view_facing_5.color.b = 255.0;

        ostringstream str5;
        str5 << "theta_heading: " << headingAngle << " deg" << ",v_angle: " << otherBerthingInfomationCalculation.v_angle << " deg/s";

        text_view_facing_5.text=str5.str();

        visualization_msgs::Marker Text_view_facing_6;
        text_view_facing_6 = Text_view_facing_6;

        //Text display Settings
        text_view_facing_6.header.frame_id = "lidar";
        text_view_facing_6.header.stamp = ros::Time::now();
        text_view_facing_6.ns = "target_text6";//namespace
        text_view_facing_6.action = visualization_msgs::Marker::ADD;
        text_view_facing_6.pose.orientation.w = 1.0;
        text_view_facing_6.id = 14; //unique id, useful when multiple markers exist.
        text_view_facing_6.type = visualization_msgs::Marker::TEXT_VIEW_FACING; //marker type
        text_view_facing_6.pose.position.x = 0;
        text_view_facing_6.pose.position.y = 0;
        text_view_facing_6.pose.position.z = 180;

        text_view_facing_6.scale.z = 20;
        text_view_facing_6.color.a = 1.0; // Don't forget to set the alpha!,the opacity
        text_view_facing_6.color.r = 160.0/255.0;
        text_view_facing_6.color.g = 32.0/255.0;
        text_view_facing_6.color.b = 240.0/255.0;

        ostringstream str6;
        str6 << "d_nearest: " << otherBerthingInfomationCalculation.d_nearest << " m";

        text_view_facing_6.text=str6.str();



    }
};


#endif //LXD_BERTHING_INFOMATION_MARKER_H
