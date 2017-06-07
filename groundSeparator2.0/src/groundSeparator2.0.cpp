#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

using namespace std;

typedef pcl::PointXYZ PointType;

void callFromThread(int id, pcl::PointCloud<pcl::PointXYZ>::Ptr cluster) {
    stringstream s;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndicesPtr ground (new pcl::PointIndices);
    std::cout << "   thread" << id << '\n';
    std::cout << "   .cluster before filtering has: " << cluster->points.size() << " points" << '\n';
    std::cout << cluster->points[0].x << '\n';
    // do the main filtering and ground extraction here
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud (cluster);
    std::cout << "   ..1" << '\n';
    pmf.setMaxWindowSize (9);
    pmf.setSlope (1.0f);
    pmf.setInitialDistance (0.25f);
    pmf.setMaxDistance (3.31f);
    std::cout << "   ..2" << '\n';
    pmf.extract (ground->indices);
    std::cout << "   ..3" << '\n';

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cluster);
    extract.setIndices (ground);
    extract.filter (*cloud_filtered);

    std::cerr << "   .." << id << "Ground cloud after filtering: " << *cloud_filtered << std::endl;

    pcl::PCDWriter writer;
    s << id;
    string filename = s.str() + "ground.pcd";
    writer.write<pcl::PointXYZ> (filename, *cloud_filtered, false);

    std::cout << "   ..4" << '\n';

}

int main (int argc, char** argv) {
    if (argc < 2) {
        return(1);
    } else {
        static const int num_threads = 4;
        thread t[num_threads];

        // load PointCloud
        string pointCloudPath = argv[1];
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

        // Fill in the cloud data
        pcl::PCDReader reader;
        reader.read<pcl::PointXYZ> (pointCloudPath, *cloud);
        int pointCloudSize = cloud->points.size();

        std::cerr << "Cloud before filtering: " << std::endl;
        std::cerr << pointCloudSize << std::endl;

        for (int i = 0; i < num_threads; i++) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<PointType>& temp_point_cloud = *temp_cloud_ptr;

            for (int j = (pointCloudSize / num_threads) * i ; j < ((pointCloudSize / num_threads) * i) + (pointCloudSize / num_threads); j++) {
                // std::cout << "point"<< j << '\n';
                // create point
                PointType point = cloud->points[j];
                // std::cout << "point:"<< point.x <<" " << cloud->points[j].x << " " << point.y <<" " << cloud->points[j].y << '\n';
                // add point
                temp_point_cloud.points.push_back(point);
            }
            std::cout << "one part" << '\n';
            temp_point_cloud.width = (int) temp_point_cloud.points.size ();  temp_point_cloud.height = 1;
            t[i] = thread(callFromThread, i, temp_cloud_ptr);

        }

        std::cout << "/* message */" << '\n';

        for (int i = 0; i < num_threads; i++) {
            t[i].join();
        }

        return(0);
    }
}
