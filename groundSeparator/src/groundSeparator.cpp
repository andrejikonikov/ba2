#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/io/ply_io.h>


using namespace std;

// typedef pcl::PointXYZ PointType;

int main (int argc, char** argv) {
    std::cout << "/* message */" << argc << '\n';

    string pointCloudPath = argv[1];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndicesPtr ground (new pcl::PointIndices);

    if (argc < 2) {
        // invalid parameters
        return(1);
    } else {
        std::cout << "/*444 message */" << '\n';
        // valid parameters
        // pcl::PLYReader reader;
        // reader.read (pointCloudPath, *cloud);
        // Fill in the cloud data
        pcl::PCDReader reader;
        reader.read (pointCloudPath, *cloud);
        // Replace the path below with the path where you saved your file
        // reader.read<pcl::PointXYZ> ("samp11-utm.pcd", *cloud);


        std::cerr << "Cloud before filtering: " << std::endl;
        std::cerr << *cloud << std::endl;

        // Create the filtering object
        pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
        pmf.setInputCloud (cloud);
        pmf.setMaxWindowSize (20);
        pmf.setSlope (1.0f);
        pmf.setInitialDistance (0.5f);
        pmf.setMaxDistance (3.0f);
        pmf.extract (ground->indices);

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud);
        extract.setIndices (ground);
        extract.filter (*cloud_filtered);

        std::cerr << "Ground cloud after filtering: " << std::endl;
        std::cerr << *cloud_filtered << std::endl;

        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZ> ("ground.pcd", *cloud_filtered, false);

        // Extract non-ground returns
        extract.setNegative (true);
        extract.filter (*cloud_filtered);

        std::cerr << "Object cloud after filtering: " << std::endl;
        std::cerr << *cloud_filtered << std::endl;

        writer.write<pcl::PointXYZ> ("object.pcd", *cloud_filtered, false);

        return (0);
    }
}
