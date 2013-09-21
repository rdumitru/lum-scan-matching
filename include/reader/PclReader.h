#ifndef _PCL_READER_H
#define _PCL_READER_H

//==============================================================================
// Includes.
//==============================================================================
#include <reader/PcReader.h>

// C++ includes.
#include <vector>

// PCL includes.
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//==============================================================================
// Class declaration.
//==============================================================================
class PclReader : public PcReader
{
private:
    // Private fields.
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> m_PointClouds;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_PcResult;

public:
    // Constructors.
    PclReader();

    PclReader(const PclReader &other);

    ~PclReader();

    // Public methods.
    void read(const std::string &path,
              const int &start = 0, const int &end = 0, const int &width = 3,
              const std::string &root = "scan", const std::string &ext = ".3d", const std::string &poseExt = ".pose");

    void run();

    void printPc(const std::string &filePath);
};

#endif // _PCL_READER_H
