//
// Created by luxiaodong on 2022/1/19.
//

#ifndef LXD_BERTHING_INFOMATION_OTHER_BERTHING_INFOMATION_CALCULATION_H
#define LXD_BERTHING_INFOMATION_OTHER_BERTHING_INFOMATION_CALCULATION_H

#include <math.h>
#include "Outlier_noise_filtering.h"



class Other_berthing_infomation_calculation{
public:
    double pai = 3.1415926 ;
    double Angle_Radians = pai/180.0;


    double vessel_slope ;
    double vessel_heading ;
    double vessel_vertical ;
    double theta_heading;
    double b_min ;
    double b_max ;
    double c_min ;
    double c_mid ;
    double c_max ;
    double h_min ;
    double h_max ;
    double d_nearest;





    double P_bow_x,P_bow_y,P_bow_z;
    double P_amidship_x,P_amidship_y,P_amidship_z;
    double P_stern_x,P_stern_y,P_stern_z;
    double P_nearest_x,P_nearest_y,P_nearest_z;

    int P_record = 0;
    double P_theta_heading;
    double P_pbow_x ,P_pbow_y,P_pbow_z;
    double P_pamidship_x,P_pamidship_y,P_pamidship_z;
    double P_pstern_x,P_pstern_y,P_pstern_z;

    double d_pbow = 0,d_pamidship = 0,d_pstern = 0;
    double d_pbow_y = 0,d_pamidship_y = 0,d_pstern_y = 0;
    double d_pamidship_x = 0;

    double v_bow = 0,v_amidship = 0,v_stern = 0;
    double v_bow_y = 0,v_amidship_y = 0,v_stern_y = 0;
    double v_amidship_x = 0;
    double v_angle = 0;



    void cal_key_point(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_stray_filtered_ptr,double heading){
        double b = 0;
        b_min = 10000.0;


        double c = 0;
        c_min = 10000.0;
        c_max = -10000.0;

        d_nearest = 10000.0;



        vessel_heading = heading * Angle_Radians;
        vessel_vertical = vessel_heading + (90.0* Angle_Radians);
        theta_heading = heading;

        if(90>heading || heading>270){

            for(int i = 0 ; i < cloud_stray_filtered_ptr->size() ; i++) {

                c = cloud_stray_filtered_ptr->points[i].y / tan(vessel_vertical)
                        - cloud_stray_filtered_ptr->points[i].x;
                if (c_min >= c) {
                    c_min = c;
                    P_bow_x = cloud_stray_filtered_ptr->points[i].x;
                    P_bow_y = cloud_stray_filtered_ptr->points[i].y;
                    P_bow_z = cloud_stray_filtered_ptr->points[i].z;
                }
                if (d_nearest >= cloud_stray_filtered_ptr->points[i].y) {
                    d_nearest = cloud_stray_filtered_ptr->points[i].y;
                    P_nearest_x = cloud_stray_filtered_ptr->points[i].x;
                    P_nearest_y = cloud_stray_filtered_ptr->points[i].y;
                    P_nearest_z = cloud_stray_filtered_ptr->points[i].z;
                }
                if (c_max <= c) {
                    c_max = c;
                }
                b = cloud_stray_filtered_ptr->points[i].y - tan(vessel_heading)*cloud_stray_filtered_ptr->points[i].x;
                if (b_min >= b){
                    b_min = b;
                }
            }
            double b1 = b_min;
            double b3 = c_max;


            P_stern_x = ( tan(vessel_vertical) * c_max - b_min) / (tan(vessel_heading) - tan(vessel_vertical));
            P_stern_y = tan(vessel_heading) * P_stern_x + b_min;
            P_stern_z = P_bow_z;

            c_mid = 0.5 * (c_min + c_max);
            P_amidship_x = ( tan(vessel_vertical) * c_mid - b_min) / (tan(vessel_heading) - tan(vessel_vertical));
            P_amidship_y = tan(vessel_heading) * P_amidship_x + b_min;
            P_amidship_z = P_bow_z;

//            cout << "heading270_90: ,heading"  << endl;
        }
        else
        {
            for(int i = 0 ; i < cloud_stray_filtered_ptr->size() ; i++) {


                c = cloud_stray_filtered_ptr->points[i].y / tan(vessel_vertical)
                    - cloud_stray_filtered_ptr->points[i].x;
                if (c_min >= c) {
                    c_min = c;

                }
                if (d_nearest >= cloud_stray_filtered_ptr->points[i].y) {
                    d_nearest = cloud_stray_filtered_ptr->points[i].y;
                    P_nearest_x = cloud_stray_filtered_ptr->points[i].x;
                    P_nearest_y = cloud_stray_filtered_ptr->points[i].y;
                    P_nearest_z = cloud_stray_filtered_ptr->points[i].z;
                }
                if (c_max <= c) {
                    c_max = c;
                    P_bow_x = cloud_stray_filtered_ptr->points[i].x;
                    P_bow_y = cloud_stray_filtered_ptr->points[i].y;
                    P_bow_z = cloud_stray_filtered_ptr->points[i].z;
                }
                b = cloud_stray_filtered_ptr->points[i].y - tan(vessel_heading)*cloud_stray_filtered_ptr->points[i].x;
                if (b_min >= b){
                    b_min = b;
                }
            }
            P_stern_x = ( tan(vessel_vertical) * c_min - b_min) / (tan(vessel_heading) - tan(vessel_vertical));
            P_stern_y = tan(vessel_heading) * P_stern_x + b_min;
            P_stern_z = P_bow_z;

            c_mid = 0.5 * (c_min + c_max);
            P_amidship_x = ( tan(vessel_vertical) * c_mid - b_min) / (tan(vessel_heading) - tan(vessel_vertical));
            P_amidship_y = tan(vessel_heading) * P_amidship_x + b_min;
            P_amidship_z = P_bow_z;

//            cout << "heading90_270: "  << heading << endl;

        }

    }

    void cal_velocity(double t){

        d_pbow = sqrt((P_pbow_x - P_bow_x) * (P_pbow_x - P_bow_x) + (P_pbow_y - P_bow_y) * (P_pbow_y - P_bow_y));
        d_pamidship = sqrt((P_pamidship_x - P_amidship_x) * (P_pamidship_x - P_amidship_x) + (P_pamidship_y - P_amidship_y) * (P_pamidship_y - P_amidship_y));
        d_pstern = sqrt((P_pstern_x - P_stern_x) * (P_pstern_x - P_stern_x) + (P_pstern_y - P_stern_y) * (P_pstern_y - P_stern_y));
        d_pbow_y = P_pbow_y - P_bow_y;
        d_pamidship_y = P_pamidship_y - P_amidship_y;
        d_pstern_y = P_pstern_y - P_stern_y;
        d_pamidship_x = P_amidship_x - P_pamidship_x;

        v_angle = (theta_heading - P_theta_heading) / t;
        v_bow = d_pbow / t;
        v_amidship = d_pamidship / t ;
        v_stern = d_pstern / t;
        v_bow_y = d_pbow_y / t;
        v_amidship_y = d_pamidship_y / t ;
        v_stern_y = d_pstern_y / t;
        v_amidship_x = d_pamidship_x / t;

    }

    void record_past_key_points(){

        P_theta_heading = theta_heading;

        P_pbow_x = P_bow_x;
        P_pamidship_x = P_amidship_x;
        P_pstern_x = P_stern_x;

        P_pbow_y = P_bow_y;
        P_pamidship_y = P_amidship_y;
        P_pstern_y = P_stern_y;

        P_pbow_z = P_bow_z;
        P_pamidship_z = P_amidship_z;
        P_pstern_z = P_stern_z;

        P_record = 1;

//        cout << "P_record : " << P_record  << endl;


    }




}
;


#endif //LXD_BERTHING_INFOMATION_OTHER_BERTHING_INFOMATION_CALCULATION_H
