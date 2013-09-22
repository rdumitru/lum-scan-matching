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
// Helpers.
//==============================================================================
enum CorrespMethod {
    ICP,
    CORRESP_EST
};

//==============================================================================
// Class declaration.
//==============================================================================
class PclReader : public PcReader
{
private:
    // Private fields.
    CorrespMethod m_CorrespMethod;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> m_PointClouds;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_PcResult;

    // Constants.
    static const int LUM_ITER = 50;
    static const float LUM_CONV_THRESH = 0.0;
    static const int ICP_ITER = 3;
    static const float ICP_MAX_CORRESP_DIST = 0.05;
    static const float ICP_TRANS_EPS = 1e-8;
    static const float ICP_EUCLIDEAN_FITNESS_EPS = 1;

public:
    // Constructors.
    PclReader(const CorrespMethod &correspMethod);

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
