#include <string>
#include <iostream>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>


using namespace std;
using namespace boost::filesystem;

typedef pcl::PointXYZ PointType;

int main (int argc, char** argv) {


    if (argc < 2) {
        return(1);
    } else {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudConjuctedPTR (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<PointType>& cloudConjucted = *cloudConjuctedPTR;


        pcl::PCDReader reader;


        path p(argv[1]);
        for (auto i = directory_iterator(p); i != directory_iterator(); i++){
            if (!is_directory(i->path())){
                cout << i->path().filename().string() << endl;
                string pointCloudPath = argv[1] + i->path().filename().string();
                reader.read<pcl::PointXYZ> (pointCloudPath, *cloud);
                int pointCloudSize = cloud->points.size();

                for (int j = 0; j < pointCloudSize; j++) {
                    std::cout << j << '\n';
                    PointType point = cloud->points[j];
                    cloudConjucted.points.push_back(point);
                }
                cloudConjucted.width = (int) cloudConjucted.points.size (); cloudConjucted.height = 1;
            }
            else
                continue;
        }

        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZ> ("ground.pcd", *cloudConjuctedPTR, false);
        pcl::PLYWriter pWriter;
        pWriter.write("ground.ply", *cloudConjuctedPTR, false);

        return(0);
    }
}
