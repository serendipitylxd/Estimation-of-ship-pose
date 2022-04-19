//
// Created by luxiaodong on 2021/12/12.
//

#ifndef LXD_BERTHING_INFOMATION_BERTHING_ANGLE_CALCULATION_H
#define LXD_BERTHING_INFOMATION_BERTHING_ANGLE_CALCULATION_H

#include "Cutting_point_cloud.h"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include<fstream>
#include<vector>
#include<string>

#include<pcl/point_types.h>
#include <math.h>

using namespace std;


using namespace Eigen;


pcl::PointCloud<pcl::PointXYZ>::Ptr Point_Cloud_Coordinate_system_transformation_ptr(new pcl::PointCloud<pcl::PointXYZ>);//去首去尾后的指针

class Berthing_angle_calculation{
public:
    double SumCalculation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr){
        sumxx = 0;
        sumxy = 0;
        sumyy = 0;
        sumx = 0;
        sumy = 0;

        for(size_t j = 0; j < cloud_ptr->points.size(); ++j)
        {

            sumx += cloud_ptr->points[j].x;
            sumy += cloud_ptr->points[j].y;
            sumxx += cloud_ptr->points[j].x * cloud_ptr->points[j].x;
            sumxy += cloud_ptr->points[j].x * cloud_ptr->points[j].y;
            sumyy += cloud_ptr->points[j].y * cloud_ptr->points[j].y;
        }

    }

    double LinerRegressionK(int n,double Xmax_Ymax){
        if (Xmax_Ymax == 1){
            k1 = (sumxy - (sumx*sumy/n))/(sumxx-(sumx*sumx/n));
        } else if  (Xmax_Ymax == 2){
            k1 = (sumxy - (sumx*sumy/n))/(sumyy-(sumy*sumy/n));
        }

        return k1;
    }
    double LinerRegressionB(int n){
        b = sumy/n - k1*sumx/n;
        return b;
    }


    void CoordinateSystemTransformation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr,double Angle){
        *Point_Cloud_Coordinate_system_transformation_ptr = *cloud_ptr;

        for(int i = 0; i < Point_Cloud_Coordinate_system_transformation_ptr->points.size(); ++i)
        {
            Point_Cloud_Coordinate_system_transformation_ptr->points[i].x =
                    cos(Angle * Angle_Radians)*cloud_ptr->points[i].x+sin(Angle * Angle_Radians) * cloud_ptr->points[i].y;
            Point_Cloud_Coordinate_system_transformation_ptr->points[i].y =
                    -sin(Angle * Angle_Radians)*cloud_ptr->points[i].x+cos(Angle * Angle_Radians) * cloud_ptr->points[i].y;

        }

    }

    // sorting
    void SortVec(const MatrixXf& mtrx,MatrixXf& sorted_mtrx,VectorXi& ind){
        ind = VectorXi::LinSpaced(mtrx.rows(),0,mtrx.rows()-1);
        auto rule=[mtrx](int i,int j)->bool
        {

            return mtrx(i,0)<mtrx(j,0);
        };
        sort(ind.data(),ind.data()+ind.size(),rule);

        sorted_mtrx.resize(mtrx.rows(),mtrx.cols());
        for(int i=0;i<mtrx.rows();i++){
            sorted_mtrx.row(i)=mtrx.row(ind(i));
        }
    }


    void AreaCalc(int num,const MatrixXf& matrix,double b1,double berthingAngle) {
        midValue = 0.5 * (matrix(0,0) + matrix(num-1,0));
        area = 0;
        area1 = 0;
        area2 = 0;
        for (int i = 1; i < num ; ++i) {
            int j = i - 1;
            area += (matrix(i,0) - matrix(j,0))* 0.5 * (matrix(i,1) + matrix(j,1)- 2 * (b1 * cos( berthingAngle * Angle_Radians)));

            if (matrix(i,0) < midValue){

            area1 +=   (matrix(i,0) - matrix(j,0))* 0.5* (matrix(i,1) + matrix(j,1)- 2 * (b1 * cos( berthingAngle * Angle_Radians)));
            } else{

            area2 +=   (matrix(i,0) - matrix(j,0)) *  0.5 * (matrix(i,1) + matrix(j,1)- 2 * (b1 * cos( berthingAngle * Angle_Radians)));
            }
        }
    }

    double area ;
    double area1 ;
    double area2 ;

private:
    double k1;
    double b;
    double sumyy;
    double sumxy;
    double sumxx;
    double sumx;
    double sumy;
    double Radians_Angle = 180.0/3.141592653;
    double Angle_Radians = 3.141592653/180.0;
    double minx ;
    double maxx ;
    double miny ;
    double maxy ;
    double midValue;

};


#endif //LXD_BERTHING_INFOMATION_BERTHING_ANGLE_CALCULATION_H
