#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>

using namespace std;

int main (int argc, char** argv) {


    if (argc < 2) {
        return(1);
    } else {
        string pointCloudPath = argv[1];
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        // Fill in the cloud data
        pcl::PCDReader reader;
        // Replace the path below with the path where you saved your file
        reader.read<pcl::PointXYZ> (pointCloudPath, *cloud);

        std::cerr << "Cloud before filtering: " << std::endl;
        std::cerr << *cloud << std::endl;

        // Create the filtering object
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        sor.filter (*cloud_filtered);

        std::cerr << "Cloud after filtering: " << std::endl;
        std::cerr << *cloud_filtered << std::endl;

        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

        sor.setNegative (true);
        sor.filter (*cloud_filtered);
        writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

        return(0);
    }




    // return (0);
}
