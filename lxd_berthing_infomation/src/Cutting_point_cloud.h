//
// Created by luxiaodong on 2021/12/11.
//

#ifndef LXD_BERTHING_INFOMATION_CUTTING_POINT_CLOUD_H
#define LXD_BERTHING_INFOMATION_CUTTING_POINT_CLOUD_H

#include "Outlier_noise_filtering.h"
#include <pcl/io/io.h>



//Pointer to point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr Cutting_current_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr Range1_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr Range2_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr Amidship_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr ShipHull_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);





class Cutting_point_cloud{
public:


    void CalcMinMaxPointXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr Point_cloud_ptr){
        LidarMinX = 1000000;
        LidarMaxX = -1000000 ;
        LidarMinY = 1000000;
        LidarMaxY = -1000000 ;
        LidarMinY = 1000000;
        LidarMaxY = -1000000 ;

        for (size_t j = 0; j < Point_cloud_ptr->points.size(); ++j){
            //X
            if(LidarMinX > Point_cloud_ptr->points[j].x)
            {LidarMinX = Point_cloud_ptr->points[j].x;};
            if(LidarMaxX < Point_cloud_ptr->points[j].x)
            {LidarMaxX = Point_cloud_ptr->points[j].x;};
            //Y
            if(LidarMinY > Point_cloud_ptr->points[j].y)
            {LidarMinY = Point_cloud_ptr->points[j].y;};
            if(LidarMaxY < Point_cloud_ptr->points[j].y)
            {LidarMaxY = Point_cloud_ptr->points[j].y;};
            //Z
            if(LidarMinZ > Point_cloud_ptr->points[j].z)
            {LidarMinZ = Point_cloud_ptr->points[j].z;};
            if(LidarMaxZ < Point_cloud_ptr->points[j].z)
            {LidarMaxZ = Point_cloud_ptr->points[j].z;};
        }

        LidarMidZ = 0.5 * (LidarMinZ + LidarMaxZ);
        LidarRangeX = LidarMaxX - LidarMinX;
        LidarRangeY = LidarMaxY - LidarMinY;

    }


    void Point_two_cutting(pcl::PointCloud<pcl::PointXYZ>::Ptr cutting_current_point_cloud_ptr,double minHeight,double midHeight, double maxHeight){

        double minHeightSubtractOne = minHeight - 0.000001 ;
        pcl::PassThrough<pcl::PointXYZ> cutting1;
        cutting1.setInputCloud (cutting_current_point_cloud_ptr);//Set the input point cloud
        cutting1.setFilterFieldName ("z");// Define the shaft
        cutting1.setFilterLimits (minHeightSubtractOne, midHeight);//　Cutting range
        // cutting1.setKeepOrganized(true); // Maintain the ordered point cloud structure
        cutting1.setFilterLimitsNegative (false);
        cutting1.filter (*Range1_cloud_ptr);

        double maxHeightAddOne = maxHeight + 0.000001 ;
        pcl::PassThrough<pcl::PointXYZ> cutting2;
        cutting2.setInputCloud (cutting_current_point_cloud_ptr);
        cutting2.setFilterFieldName ("z");
        cutting2.setFilterLimits (midHeight, maxHeightAddOne);
        // cutting2.setKeepOrganized(true);
        cutting2.setFilterLimitsNegative (false);
        cutting2.filter (*Range2_cloud_ptr);
    }

    int rangePointNumCal(pcl::PointCloud<pcl::PointXYZ>::Ptr cutting_current_point_cloud_ptr){
        rangePointNum = cutting_current_point_cloud_ptr->points.size();
        return rangePointNum;
    }

    int range1PointNumCal(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_range1_ptr){
        range1PointNum = point_cloud_range1_ptr->points.size();
        return range1PointNum;
    }

    int range2PointNumCal(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_range2_ptr){
        range2PointNum = point_cloud_range2_ptr->points.size();
        return range2PointNum;
    }

    void XamidshipFiltering(pcl::PointCloud<pcl::PointXYZ>::Ptr cutting_current_point_cloud_ptr,double minX,double maxX){

        double x_range = maxX-minX;
        double x_amidship_min = minX + (9 * x_range / 19);
        double x_amidship_max = maxX - (9 * x_range / 19);
        pcl::PassThrough<pcl::PointXYZ> pass3;
        pass3.setInputCloud (cutting_current_point_cloud_ptr);
        pass3.setFilterFieldName ("x");
        pass3.setFilterLimits (x_amidship_min, x_amidship_max);
        // pass.setKeepOrganized(true);
        pass3.setFilterLimitsNegative (false);
        pass3.filter (*Amidship_cloud_ptr);
    }

    void YamidshipFiltering(pcl::PointCloud<pcl::PointXYZ>::Ptr cutting_current_point_cloud_ptr,double minY,double maxY){

        double y_range = maxY-minY;
        double y_amidship_min = minY + (9 * y_range / 19);
        double y_amidship_max = maxY - (9 * y_range / 19);
        pcl::PassThrough<pcl::PointXYZ> pass4;
        pass4.setInputCloud (cutting_current_point_cloud_ptr);
        pass4.setFilterFieldName ("y");
        pass4.setFilterLimits (y_amidship_min, y_amidship_max);
        // pass.setKeepOrganized(true);
        pass4.setFilterLimitsNegative (false);
        pass4.filter (*Amidship_cloud_ptr);
    }

    void Point_hull_cutting(pcl::PointCloud<pcl::PointXYZ>::Ptr cutting_current_point_cloud_ptr,double minHeight,double maxHeight){

        double minHeightSubtractOne = minHeight - 0.000001 ;
        double maxHeightAddOne = maxHeight + 0.000001 ;
        pcl::PassThrough<pcl::PointXYZ> cutting1;
        cutting1.setInputCloud (cutting_current_point_cloud_ptr);
        cutting1.setFilterFieldName ("z");
        cutting1.setFilterLimits (minHeightSubtractOne, maxHeightAddOne);
        // cutting1.setKeepOrganized(true);
        cutting1.setFilterLimitsNegative (false);
        cutting1.filter (*ShipHull_cloud_ptr);

    }


    double LidarRangeX;
    double LidarRangeY;
    double LidarMinX;
    double LidarMaxX;
    double LidarMinY;
    double LidarMaxY;
    double LidarMinZ;
    double LidarMidZ;
    double LidarMaxZ;

private:

    int range1PointNum;
    int range2PointNum;
    int rangePointNum;




};




#endif //LXD_BERTHING_INFOMATION_CUTTING_POINT_CLOUD_H
