#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <vector>

#include <typeinfo>

using namespace std;

typedef pcl::PointXYZ PointType;

class Cell {
    public:
        Cell(int x, int y){
            this->x = x;
            this->y = y;
        };
        int x, y;
};

class Candidate {
    public:
        Candidate(){
            fields = 0;
            x1 = x2 = y1 = y2 = 0;
        };
        void addCell(int x, int y, float minX, float minY, float step) {
            // std::cout << "   addCell" << x << " " << y << " " << minX << " " << minY << " " << step << '\n';
            if (!this->fields) {
                this->x1 = minX + (x * step);
                this->y1 = minY + (y * step);
                this->x2 = minX + (step * x) + step;
                this->y2 = minY + (step * y) + step;
                // std::cout << "    addCell" << '\n';
            } else {
                this->x2 = minX + (step * x) + step;
                this->y2 = minY + (step * y) + step;
                // std::cout << "    addCell increased" << '\n';
            }
            this->cels.push_back(Cell(x, y));
            this->fields++;
        };
        void showCells(){
            for (int i = 0;  i < this->cels.size(); i++) {
                std::cout << "   cell:"<< this->cels[i].x << " " << this->cels[i].y << '\n';
            }
            std::cout << this->x1 << " " << this->y1 << " " << this->x2 << " " << this->y2 << '\n';
        };
        int getFields(){
            return(this->fields);
        };
        float getX1(){
            return(this->x1);
        };
        float getX2(){
            return(this->x2);
        };
        float getY1(){
            return(this->y1);
        };
        float getY2(){
            return(this->y2);
        }
    private:
        std::vector<Cell> cels;
        int fields;
        float x1, x2, y1, y2;
};

int main (int argc, char** argv) {
    if (argc < 2) {
        return(1);
    } else {
        string pointCloudPath = argv[1];
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPTR (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<PointType>& cloud = *cloudPTR;
        pcl::PCDReader reader;

        reader.read<pcl::PointXYZ> (pointCloudPath, cloud);
        float minX, maxX, minY, maxY;
        minX = maxX = cloud.points[0].x;
        minY = maxY = cloud.points[0].y;
        int pointCloudSize = cloud.points.size();

        for (int i = 0; i < pointCloudSize; i++) {
            if(minX > cloud.points[i].x) {
                minX = cloud.points[i].x;
            } else {
                if(maxX < cloud.points[i].x) {
                    maxX = cloud.points[i].x;
                }
            }
            if(minY > cloud.points[i].y) {
                minY = cloud.points[i].y;
            } else {
                if(maxY < cloud.points[i].y) {
                    maxY = cloud.points[i].y;
                }
            }
        }

        float cloudLength = maxX - minX;

        std::cout << "minX=" << minX << " maxX="<< maxX << " len="<< cloudLength <<'\n';
        std::cout << "minY=" << minY << " maxY="<< maxY << " len="<< cloudLength <<'\n';

        float step = 1;
        float candidateSize = 3; // in meters???
        int xIntervals = (maxX - minX) / step;
        int yIntervals = (maxY - minY) / step;
        int m[xIntervals][yIntervals];

        std::cout << "xIntervals:" << xIntervals<< " yIntervals:"<< yIntervals << '\n';

        for (int i = 0; i < xIntervals; i++) {
            for (int k = 0; k < yIntervals; k++) {
                // std::cout << "/* message */"<< i << " " << k << '\n';
                pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<PointType>& temp_point_cloud = *temp_cloud_ptr;

                for (int j = 0; j < pointCloudSize; j++) {
                    if ((cloud.points[j].x >= minX + (step * i)) && (cloud.points[j].x < minX + (step * i) + step) && (cloud.points[j].y >= minY + (step * k)) && (cloud.points[j].y < minY + (step * k) + step) ) {
                        PointType point = cloud.points[j];
                        temp_point_cloud.points.push_back(point);
                    }
                }
                // std::cout << "one part" << '\n';
                temp_point_cloud.width = (int) temp_point_cloud.points.size ();  temp_point_cloud.height = 1;

                // std::cout << temp_point_cloud.points.size () << std::endl;
                if (temp_point_cloud.points.size()) {
                    m[i][k] = 1;
                    stringstream s;
                    pcl::PCDWriter writer;
                    s << i << k;
                    string filename = "voxels/" + s.str() + "ground.pcd";
                    writer.write<pcl::PointXYZ> (filename, temp_point_cloud, false);
                } else {
                    m[i][k] = 0;
                }
            }
        }

        for (int i = 0; i < xIntervals ; i++) {
            for (int j = 0; j < yIntervals; j++) {
                std::cout << m[i][j];
            }
            std::cout << '\n';
        }

        // ok END

        vector<Candidate> candidates;
        vector<int> values;
        vector<pcl::PointCloud<pcl::PointXYZ>>  candidateClouds;
        vector<pcl::PointCloud<pcl::PointXYZ>>  approvedClouds;

        for (int i = 0; i < xIntervals ; i++) {
            for (int j = 0; j < yIntervals; j++) {
                if (m[i][j]) {
                    // std::cout << "i="<< i << "j=" << j <<'\n';
                    if (i + candidateSize - 1 < xIntervals) {
                        if (j + candidateSize - 1 < yIntervals) {
                            Candidate c;
                            for (int x = i; x < i + candidateSize; x++) {
                                for (int y = j; y < j + candidateSize; y++) {
                                    if (m[x][y]) {
                                        // std::cout << "   x=" << x << "y=" << y << '\n';
                                        c.addCell(x, y, minX, minY, step);
                                    }
                                }
                            }
                            candidates.push_back(c);
                        }
                    }
                }
            }
        }

        std::cout << "candidates1:"<< candidates.size() << '\n';

        // remove candidates with not enough voxels
        for (int i = 0;  i < candidates.size(); i++) {
            // candidates[i].showCells();
            // std::cout << candidates[i].getFields()<< '\n';
            if (candidates[i].getFields() < candidateSize * candidateSize ) {
                candidates.erase(candidates.begin() + i);
            }
        }
        std::cout << "candidates2:"<< candidates.size() << '\n';
        // // get values of candidates
        for (int i = 0;  i < candidates.size(); i++) {
            pcl::PointCloud<pcl::PointXYZ> tcloud;

            for (int j = 0; j < pointCloudSize; j++) {
                if (
                    (cloud.points[j].x >= candidates[i].getX1()) &&
                    (cloud.points[j].x < candidates[i].getX2()) &&
                    (cloud.points[j].y >= candidates[i].getY1()) &&
                    (cloud.points[j].y < candidates[i].getY2())
                ) {
                    PointType point = cloud.points[j];
                    tcloud.points.push_back(point);
                }
            }
            tcloud.width = (int) tcloud.points.size (); tcloud.height = 1;
            candidateClouds.push_back(tcloud);
            values.push_back( (int) tcloud.points.size ());
            // std::cout << "pushing value:"<< (int) tcloud.points.size () << '\n';
        }
        std::cout << "candidates3:"<< candidates.size() << '\n';
        std::cout << "candidatesclouds3:"<< candidateClouds.size() << '\n';

        int average = accumulate( values.begin(), values.end(), 0.0)/values.size();
        int stdDev = sqrt(inner_product(values.begin(), values.end(), values.begin(), 0.0) / values.size() - average * average);
        int treshold = average;
        // int treshold = static_cast<int>(ttreshold);
        cout << "The average is" << average << "The stdDev is" << stdDev << " treshold="<< treshold << typeid(treshold).name() << endl;

        // remove candidates with less points than treshold
        for (int i = 0; i < candidateClouds.size(); i++) {
            if (candidateClouds[i].points.size() >= treshold) {
                // std::cout << "picked cloud with value:" << candidateClouds[i] << '\n';
                approvedClouds.push_back(candidateClouds[i]);

            }
        }
        //
        std::cout << "candidates4:"<< approvedClouds.size() << '\n';

        // save appproved candidates

        for (int i = 0;  i < approvedClouds.size(); i++) {
            // std::cout << approvedClouds[i] << '\n';

            stringstream s;
            pcl::PCDWriter writer;
            s << i;
            string filename = "candidates/" + s.str() + "ground.pcd";
                writer.write<pcl::PointXYZ> (filename, approvedClouds[i], false);
        }

        return(0);
    }
}
