//==============================================================================
// Includes.
//==============================================================================
#include <iostream>
using namespace std;

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/lum.h>

//==============================================================================
// Main.
//==============================================================================
int main(int argc, char* argv[]) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::io::

    pcl::registration::LUM<pcl::PointXYZ> lum;

    cout << "Program end..." << endl;
}

