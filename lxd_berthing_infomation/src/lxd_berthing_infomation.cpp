#include <pcl/io/pcd_io.h>//which contains the required definitions to load and store point clouds to PCD and other file formats.
#include <sys/time.h>
#include <string>
#include "fstream"
#include "Outlier_noise_filtering.h"
#include "Cutting_point_cloud.h"
#include "Berthing_angle_calculation.h"
#include "Other_berthing_infomation_calculation.h"
#include "Marker.h"

int main (int argc, char **argv)
{
    ros::init (argc, argv, "lxd_berthing_infomation");
    ros::NodeHandle nh;
    //Defining publishers
    ros::Publisher pcl_pub_1 = nh.advertise<sensor_msgs::PointCloud2> ("/lxd_pointcloud_output", 10);

    ros::Publisher marker_pub_1 = nh.advertise<visualization_msgs::Marker>("/lxd_Line_visualization_marker", 10);

    ros::Publisher marker_pub_2 = nh.advertise<visualization_msgs::Marker>("/lxd_ship_keyPoint_visualization_marker", 10);

    ros::Publisher text_pub_1 = nh.advertise<visualization_msgs::Marker>("/lxd_text_marker", 10);


    sensor_msgs::PointCloud2 output1;

    int year, month, day;
    int hour, minute;
    double second;
    // Initialize the past time
    int  pyear = 0, pmonth = 0, pday = 0;
    int phour = 0, pminute = 0;
    double psecond = 0 ;
    // Time difference t
    double t;
    FILE* TimeFile;
    // Change it to its own file path
    string timeStamps ="/home/luxiaodong/lxd_ws/src/lxd_berthing_infomation/data/lidar_berthing/timestamps.txt";
    TimeFile = std::fopen(timeStamps.c_str() , "r");

    Other_berthing_infomation_calculation otherBerthingInfomationCalculation;

    // Change it to its own file path
    ofstream out("/home/luxiaodong/lxd_ws/src/lxd_berthing_infomation/data/output/lxd_output.txt");
    out << "t" << "\t" << "theta_heading" << "\t" <<  "v_angle" << "\t" << "d_nearest" << "\t"
        <<  "d_bow" << "\t" <<  "v_bow" << "\t" <<  "v_ybow" << "\t"
        << "d_amidship" << "\t" << "v_amidship" << "\t"<< "v_yamidship" << "\t"
        <<  "d_stern" << "\t" <<  "v_stern" << "\t" <<  "v_ystern" << "\t"
        << "d_boffset" << "\t" << "v_xamidship" << "\n" ;

    for(int i = 0;i < 81;i = i + 1 ){

        // Read the timestamp
        fscanf(TimeFile, "%d-%d-%d %d:%d:%lf", &year, &month, &day, &hour, &minute, &second);
        cout << "year: " << year << ",month: " << month << ",day: " << day << ",hour: " << hour << ",minute: " << minute << ",second: " << second <<endl;
        if (pyear != 0){
            t = second - psecond ;
            t += 60 * (minute - pminute) ;
            t += 60 * 60 * (hour - phour);
            t += 24 * 60 * 60 * (day - pday);
            if (pyear % 4 == 0 ){
                if (pmonth == 1 || pmonth == 3 || pmonth == 5 || pmonth == 7 || pmonth == 8 || pmonth == 10 || pmonth == 12){
                    t += 31 * 24 * 60 * 60 * (month - pmonth);
                } else if (pmonth == 2) {
                    t += 29 * 24 * 60 * 60 * (month - pmonth);
                } else {
                    t += 30 * 24 * 60 * 60 * (month - pmonth);
                }

                t += 366 * 24 * 60 * 60 * (year - pyear);

            }else
            {
                if (pmonth == 1 || pmonth == 3 || pmonth == 5 || pmonth == 7 || pmonth == 8 || pmonth == 10 || pmonth == 12){
                    t += 31 * 24 * 60 * 60 * (month - pmonth);
                } else if (pmonth == 2) {
                    t += 28 * 24 * 60 * 60 * (month - pmonth);
                } else {
                    t += 30 * 24 * 60 * 60 * (month - pmonth);
                }

                t += 365 * 24 * 60 * 60 * (year - pyear);
            }
            cout << "t: " << t << " s" <<endl;
        }
        pyear=year;
        pmonth=month;
        pday=day;
        phour=hour;
        pminute=minute;
        psecond=second;



        std::stringstream ss1;//Lidar1 data flow
        std::stringstream ss2;//Lidar2 data flow
        // Change it to its own file path
        ss1 << "/home/luxiaodong/lxd_ws/src/lxd_berthing_infomation/data/lidar_berthing/Lidar1/" << i <<"lidar1.pcd";
        ss2 << "/home/luxiaodong/lxd_ws/src/lxd_berthing_infomation/data/lidar_berthing/Lidar2/" << i <<"lidar2.pcd";


        Cutting_point_cloud cuttingPointCloud;
        // Import the Lidar point cloud
        pcl::io::loadPCDFile (ss1.str(), *PCD_Lidar1_ptr);
        pcl::io::loadPCDFile (ss2.str(), *PCD_Lidar2_ptr);

        //The point cloud data of two lidars are spliced into one point cloud data
        *PCD_Lidar_ptr = *PCD_Lidar1_ptr;
        *PCD_Lidar_ptr += *PCD_Lidar2_ptr;



        //Remove the point cloud outside the port berth
        Outliner_noise_filtering outlinerNoiseFiltering;

        outlinerNoiseFiltering.Port_filtering(PCD_Lidar1_ptr);
        *Lidar1_cloud_port_filtered_ptr  = *cloud_port_filtered_ptr;
        outlinerNoiseFiltering.Port_filtering(PCD_Lidar2_ptr);
        *Lidar2_cloud_port_filtered_ptr  = *cloud_port_filtered_ptr;

        //Point cloud noise reduction
        outlinerNoiseFiltering.RadiusOutlierRemoval_filtering(Lidar1_cloud_port_filtered_ptr);
        *Lidar1_cloud_stray_filtered_ptr = *cloud_stray_filtered_ptr;
        outlinerNoiseFiltering.RadiusOutlierRemoval_filtering(Lidar2_cloud_port_filtered_ptr);
        *Lidar2_cloud_stray_filtered_ptr = *cloud_stray_filtered_ptr;
/*        outlinerNoiseFiltering.RadiusOutlierRemoval_filtering1(Lidar1_cloud_port_filtered_ptr);
        outlinerNoiseFiltering.RadiusOutlierRemoval_filtering2(Lidar2_cloud_port_filtered_ptr);*/

        //The point cloud data of two lidars are spliced into one point cloud data
        *Lidar_cloud_stray_filtered_ptr = *Lidar1_cloud_stray_filtered_ptr;
        *Lidar_cloud_stray_filtered_ptr += *Lidar2_cloud_stray_filtered_ptr;


        //Determine the rangexy of the point cloud
        double rangeX;
        double rangeY;
        double LidarminX;
        double LidarmaxX;
        double Lidar1minY;
        double Lidar1maxY;
        double Lidar2minY;
        double Lidar2maxY;


        cuttingPointCloud.CalcMinMaxPointXYZ(Lidar_cloud_stray_filtered_ptr);
        rangeX = cuttingPointCloud.LidarRangeX;
        rangeY = cuttingPointCloud.LidarRangeY;
        LidarminX = cuttingPointCloud.LidarMinX;
        LidarmaxX = cuttingPointCloud.LidarMaxX;

//        cout << "rangeX: " << rangeX << " rangeY: " << rangeY << endl;

        double minHeight;
        double midHeight;
        double maxHeight;
        double range;

        minHeight = cuttingPointCloud.LidarMinZ;
        maxHeight = cuttingPointCloud.LidarMaxZ;
        range = maxHeight - minHeight;
        maxHeight = maxHeight - 0.66 * range;//Take the ship under a third of the cloud cut
        midHeight = 0.5 * (minHeight + maxHeight);

        int rangeNum = 1000000;
        int range1Num,range2Num;
        int Xmax_Ymax = 0;
        int lidar1Y_lidar2Y = 0;
        int lidar1_lidar2 = 0;
        string rangx_rangy;


        if (rangeX > rangeY){

            Xmax_Ymax = 1;
            rangx_rangy = "rangex";
            *Cutting_current_cloud_ptr = *Lidar_cloud_stray_filtered_ptr;

        }
        else {
            rangx_rangy = "rangey";
            Xmax_Ymax = 2;
            double Lidar1rangeY;
            double Lidar1num;
            cuttingPointCloud.CalcMinMaxPointXYZ(Lidar1_cloud_stray_filtered_ptr);
            Lidar1rangeY = cuttingPointCloud.LidarRangeY;
            Lidar1num = Lidar1_cloud_stray_filtered_ptr->size();

            Lidar1minY = cuttingPointCloud.LidarMinY;
            Lidar1maxY = cuttingPointCloud.LidarMaxY;
//            cout << "Lidar1minY: " << Lidar1minY << " Lidar1maxY: " << Lidar1maxY << endl;
            double Lidar2rangeY;



            // Handle the point cloud with a quantitative determination
            double Lidar2num;

            cuttingPointCloud.CalcMinMaxPointXYZ(Lidar2_cloud_stray_filtered_ptr);
            Lidar2rangeY = cuttingPointCloud.LidarRangeY;

            Lidar2minY = cuttingPointCloud.LidarMinY;
            Lidar2maxY = cuttingPointCloud.LidarMaxY;

            Lidar2num = Lidar2_cloud_stray_filtered_ptr->size();
//            cout << "Lidar2minY: " << Lidar2minY << " Lidar2maxY: " << Lidar2maxY << endl;
            //cout << "Lidar1rangeY:" << Lidar1rangeY << " Lidar2rangeY:" << Lidar2rangeY <<endl;

            if(Lidar1num > Lidar2num){
                lidar1_lidar2 = 1;
                *Cutting_current_cloud_ptr = *Lidar1_cloud_stray_filtered_ptr;
            }
            else{
                lidar1_lidar2 = 2;
                *Cutting_current_cloud_ptr = *Lidar2_cloud_stray_filtered_ptr;
            }


        }

        // Capture the point cloud data near the ship
        if (Xmax_Ymax == 1){
            //Cutting_current_cloud_ptr is the fused data
            cuttingPointCloud.XamidshipFiltering(Cutting_current_cloud_ptr,LidarminX,LidarmaxX);
        } else if (Xmax_Ymax == 2){
            if (lidar1_lidar2 == 1){
//                cout << "Lidar1minY: " << Lidar1minY << " Lidar1maxY: " << Lidar1maxY << endl;
                //Cutting_current_cloud_ptr is Lidar1 data
                cuttingPointCloud.YamidshipFiltering(Cutting_current_cloud_ptr,Lidar1minY,Lidar1maxY);
            } else if (lidar1_lidar2 == 2){
                //Cutting_current_cloud_ptr is Lidar2 data
//                cout << "Lidar2minY: " << Lidar2minY << " Lidar2maxY: " << Lidar2maxY << endl;
                cuttingPointCloud.YamidshipFiltering(Cutting_current_cloud_ptr,Lidar2minY,Lidar2maxY);

            }
        }



        while (range > 1 && rangeNum > 50){
//        while ( rangeNum > 50){


            cuttingPointCloud.Point_two_cutting(Amidship_cloud_ptr,minHeight,midHeight,maxHeight);
            range1Num = cuttingPointCloud.range1PointNumCal(Range1_cloud_ptr);


            range2Num = cuttingPointCloud.range2PointNumCal(Range2_cloud_ptr);


            if (range1Num > range2Num){

                *Amidship_cloud_ptr = *Range1_cloud_ptr;
                maxHeight = midHeight;
                midHeight = 0.5 * (minHeight + maxHeight );
            }
            else{
                *Amidship_cloud_ptr = *Range2_cloud_ptr;
                minHeight = midHeight;
                midHeight = 0.5 * (minHeight + maxHeight );
            }

            range = maxHeight - minHeight;

            rangeNum = cuttingPointCloud.rangePointNumCal(Amidship_cloud_ptr);
//            cout << "range " << range << ", rangeNum " << rangeNum << endl;
        }
//

        //Calculate the theta_berthing
        double k1;
        double b1;
        double Radians_Angle = 180.0/3.141592653;
        double berthingAngle;
        Berthing_angle_calculation berthingAngleCalculation;
        berthingAngleCalculation.SumCalculation(Amidship_cloud_ptr);
        k1 = berthingAngleCalculation.LinerRegressionK(Amidship_cloud_ptr->points.size(), Xmax_Ymax);

        b1 = berthingAngleCalculation.LinerRegressionB(Amidship_cloud_ptr->points.size());

        if (Xmax_Ymax == 1){
            berthingAngle = Radians_Angle * atan(k1);
        } else if (Xmax_Ymax == 2 ){
            double YberthingAngle;
//            cout << "---------------------------" << endl;
            YberthingAngle = Radians_Angle * atan(k1);
            berthingAngle = -(YberthingAngle - 90);
            if  (berthingAngle > 90){
                berthingAngle = berthingAngle -180 ;
            }

        }

       cout << "theta_berthing " << berthingAngle << "°" << "k1:" << k1 << endl;


        //Coordinate system transformation
        cuttingPointCloud.Point_hull_cutting(Cutting_current_cloud_ptr,minHeight,maxHeight);
        berthingAngleCalculation.CoordinateSystemTransformation(ShipHull_cloud_ptr,berthingAngle);



        //Point cloud sorting
        MatrixXf matrix_a,matrix_a_sort;
        VectorXi ind;
        int DynamicRows = Point_Cloud_Coordinate_system_transformation_ptr->size();

        matrix_a.resize(DynamicRows,3);

        for (int i = 0; i < Point_Cloud_Coordinate_system_transformation_ptr->size(); ++i) {
        matrix_a(i,0) = Point_Cloud_Coordinate_system_transformation_ptr->points[i].x;
        matrix_a(i,1) = Point_Cloud_Coordinate_system_transformation_ptr->points[i].y;
        matrix_a(i,2) = i;
        }

        berthingAngleCalculation.SortVec(matrix_a,matrix_a_sort,ind);


        // calculate Area1 and Area2

        double area;
        double area1;
        double area2;
        berthingAngleCalculation.AreaCalc(Point_Cloud_Coordinate_system_transformation_ptr->size(),matrix_a_sort,b1,berthingAngle);
        area = berthingAngleCalculation.area;
        area1 = berthingAngleCalculation.area1;
        area2 = berthingAngleCalculation.area2;

//        cout << "area: " << area << " area1: " << area1 << " area2: " << area2 << endl;

        // Calculate the theta_heading
        double headingAngle;

        if (berthingAngle > 0 ){
            if (Xmax_Ymax == 1 || lidar1_lidar2 == 2){
                if (area2 > area1){
                    headingAngle = berthingAngle;
                }else{
                    headingAngle = berthingAngle + 180;
                }
            } else if (lidar1_lidar2 == 1){
                if (area1 > area2){
                    headingAngle = berthingAngle;
                }else{
                    headingAngle = berthingAngle + 180;
                }
            }
        } else{
            if (Xmax_Ymax == 1 || lidar1_lidar2 == 1){
                if (area2 > area1){
                    headingAngle = berthingAngle;
                }else{
                    headingAngle = berthingAngle + 180;
                }
            } else if (lidar1_lidar2 == 2){
                if (area1 > area2){
                    headingAngle = berthingAngle;
                }else{
                    headingAngle = berthingAngle + 180;
                }
            }
        }


//        Other_berthing_infomation_calculation otherBerthingInfomationCalculation;

        otherBerthingInfomationCalculation.cal_key_point(Lidar_cloud_stray_filtered_ptr,headingAngle);
        if (otherBerthingInfomationCalculation.P_record == 0 ){
//            cout << "v_bow_y: " << otherBerthingInfomationCalculation.v_bow_y
//                 << " m/s,v_amidship_y: " << otherBerthingInfomationCalculation.v_amidship_y
//                 << " m/s,v_stern_y: " << otherBerthingInfomationCalculation.v_stern_y
//                 << " m/s,v_amidship_x: " << otherBerthingInfomationCalculation.v_amidship_x << " m/s"
//                 << endl;
        } else
        {
            otherBerthingInfomationCalculation.cal_velocity(t);
//            cout << "v_bow_y: " << otherBerthingInfomationCalculation.v_bow_y
//                 << " m/s,v_amidship_y: " << otherBerthingInfomationCalculation.v_amidship_y
//                 << " m/s,v_stern_y: " << otherBerthingInfomationCalculation.v_stern_y
//                 << " m/s,v_amidship_x: " << otherBerthingInfomationCalculation.v_amidship_x << " m/s"
//                 << endl;
        }
        otherBerthingInfomationCalculation.record_past_key_points();

        Draw_marker drawMarker;
        drawMarker.draw_line(otherBerthingInfomationCalculation);
        drawMarker.draw_sphere(otherBerthingInfomationCalculation);
        drawMarker.draw_text(otherBerthingInfomationCalculation,headingAngle);


        pcl::toROSMsg(*PCD_Lidar_ptr, output1);


        //Note that this step is rviz fixed_frame
       output1.header.frame_id = "lidar";//this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer



            pcl_pub_1.publish(output1);

            marker_pub_1.publish(line_strip_1);
            marker_pub_1.publish(line_strip_2);
            marker_pub_1.publish(line_strip_3);
            marker_pub_1.publish(line_strip_4);
            marker_pub_1.publish(line_strip_5);
            marker_pub_2.publish(sphere_stern);
            marker_pub_2.publish(sphere_amidship);
            marker_pub_2.publish(sphere_bow);


            text_pub_1.publish(text_view_facing_1);
            text_pub_1.publish(text_view_facing_2);
            text_pub_1.publish(text_view_facing_3);
            text_pub_1.publish(text_view_facing_4);
            text_pub_1.publish(text_view_facing_5);
            text_pub_1.publish(text_view_facing_6);

            ros::spinOnce();


        out << t << "\t" << otherBerthingInfomationCalculation.theta_heading << "\t" << otherBerthingInfomationCalculation.v_angle << "\t" << otherBerthingInfomationCalculation.d_nearest << "\t"
            << otherBerthingInfomationCalculation.P_bow_y << "\t" << otherBerthingInfomationCalculation.v_bow << "\t" << otherBerthingInfomationCalculation.v_bow_y << "\t"
            << otherBerthingInfomationCalculation.P_amidship_y << "\t" << otherBerthingInfomationCalculation.v_amidship << "\t" << otherBerthingInfomationCalculation.v_amidship_y << "\t"
            << otherBerthingInfomationCalculation.P_stern_y << "\t" << otherBerthingInfomationCalculation.v_stern << "\t" << otherBerthingInfomationCalculation.v_stern_y << "\t"
            << otherBerthingInfomationCalculation.P_amidship_x << "\t" << otherBerthingInfomationCalculation.v_amidship_x << "\t"
            << "\n";


        // Point cloud loop
    }
    out.close();
    fclose(TimeFile);

    return 0;
}
