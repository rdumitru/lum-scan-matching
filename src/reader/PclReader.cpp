//==============================================================================
// Includes.
//==============================================================================
#include <reader/PclReader.h>

// User includes.
#include <common.h>
#include <timer/Timer.h>

// C++ includes.
#include <iostream>
#include <fstream>
#include <string>
using namespace std;

// C includes.
#include <assert.h>
#include <time.h>

// PCL includes.
#include <pcl/registration/lum.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>

//==============================================================================
// Class implementation.
//==============================================================================
// Constructors.
PclReader::PclReader(const CorrespMethod &correspMethod) : PcReader()
{
    this->m_PointClouds.clear();
    this->m_CorrespMethod = correspMethod;
}

PclReader::PclReader(const PclReader &other) : PcReader(other)
{
    this->m_PointClouds = other.m_PointClouds;
    this->m_PcResult = other.m_PcResult;
    this->m_CorrespMethod = other.m_CorrespMethod;
}

PclReader::~PclReader()
{}

// Public methods.
void PclReader::read(const string &path,
                     const int &start, const int &end, const int &width,
                     const string &root, const string &ext, const string &poseExt)
{
    Timer timer;
    timer.start();

    cout << "Reading scans..." << endl;
    assert(start >= 0);
    assert(start <= end);
    assert(width > 0);

    // Make sure that we first clear all the previously loaded point clouds.
    this->m_PointClouds.clear();

    // Go from start to end and read point clouds.
    for (int it = start; it <= end; ++it)
    {
        timer.record();

        string fileRoot = root + int2String(it, width);
        string fullPath = path;

        if (*path.rbegin() != this->fileSep)
        {
            fullPath += this->fileSep;
        }

        fullPath += fileRoot;

        string pcFilePath = fullPath + ext;
        string poseFilePath = fullPath + poseExt;

        cout << "Loading " << pcFilePath << "..." << endl;

        // Open the stream to the pc file.
        ifstream pcFile(pcFilePath.c_str());
        if (pcFile.is_open() == false) {
            die("Failed to open file \"" + pcFilePath + "\"...");
        }

        // Read in the point cloud.
        pcl::PointCloud<pcl::PointXYZ>::Ptr p_Pc(new pcl::PointCloud<pcl::PointXYZ>);

        double x, y, z;
        while (!pcFile.eof())
        {
            pcl::PointXYZ pt;
            pcFile >> pt.x >> pt.y >> pt.z;
            p_Pc->points.push_back(pt);
        }

        // Open the stream to the pose file.
        ifstream poseFile(poseFilePath.c_str());
        if (poseFile.is_open() == false) {
            die("Failed to open file \"" + poseFilePath + "\"...");
        }

        pcFile.close();

        // Read in the pose.
        Pose pose;
        poseFile >> pose.x >> pose.y >> pose.z;
        poseFile >> pose.roll >> pose.pitch >> pose.yaw;

        // Convert from degrees to radians.
        pose.roll = deg2Rad(pose.roll);
        pose.pitch = deg2Rad(pose.pitch);
        pose.yaw = deg2Rad(pose.yaw);

        // Add the point cloud and pose to the list.
        this->m_PointClouds.push_back(p_Pc);
        this->m_Poses.push_back(pose);

        poseFile.close();

        cout << "Loaded point cloud with " << p_Pc->points.size() << " points @pose("
                    << pose.x << ", " << pose.y << ", " << pose.z << "; "
             << pose.roll << ", " << pose.pitch << ", " << pose.yaw
             << ")..." << endl;

        timer.record();
        timer.printTime("Point cloud load");
    }
}

void PclReader::run()
{
    Timer timer;
    timer.start();

    cout << "Running PCL Lu and Milios Scan Matching algorithm..." << endl;
    assert(this->m_PointClouds.size() == this->m_Poses.size());

    pcl::registration::LUM<pcl::PointXYZ> lum;

    // Add SLAM Graph vertices.
    for (int it = 0; it < this->m_PointClouds.size(); ++it)
    {
        Eigen::Vector6f pose;
        pose << this->m_Poses[it].x, this->m_Poses[it].y, this->m_Poses[it].z,
                this->m_Poses[it].roll, this->m_Poses[it].pitch, this->m_Poses[it].yaw;

        lum.addPointCloud(this->m_PointClouds[it], pose);
    }

    // Add the correspondence results as edges to the SLAM graph.
    timer.record();

    if (this->m_CorrespMethod == ICP) {
        for (int it = 0; it < this->m_PointClouds.size() - 1; ++it) {
            size_t src = it;
            size_t dst = it + 1;

            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setMaximumIterations(this->ICP_ITER);
            icp.setMaxCorrespondenceDistance(this->ICP_MAX_CORRESP_DIST);
            icp.setTransformationEpsilon(this->ICP_TRANS_EPS);
            icp.setEuclideanFitnessEpsilon(this->ICP_EUCLIDEAN_FITNESS_EPS);

            icp.setInputSource(this->m_PointClouds[src]);
            icp.setInputTarget(this->m_PointClouds[dst]);

            pcl::PointCloud<pcl::PointXYZ> temp;
            icp.align(temp);

            pcl::CorrespondencesPtr icpCorresp = icp.correspondences_;
            lum.setCorrespondences(src, dst, icpCorresp);
        }

        // Close the loop.
        size_t src = this->m_PointClouds.size() - 1;
        size_t dst = 0;

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setMaximumIterations(this->ICP_ITER);
        icp.setMaxCorrespondenceDistance(this->ICP_MAX_CORRESP_DIST);
        icp.setTransformationEpsilon(this->ICP_TRANS_EPS);
        icp.setEuclideanFitnessEpsilon(this->ICP_EUCLIDEAN_FITNESS_EPS);

        icp.setInputSource(this->m_PointClouds[src]);
        icp.setInputTarget(this->m_PointClouds[dst]);

        pcl::PointCloud<pcl::PointXYZ> temp;
        icp.align(temp);

        pcl::CorrespondencesPtr icpCorresp = icp.correspondences_;
        lum.setCorrespondences(src, dst, icpCorresp);

    } else if (this->m_CorrespMethod == CORRESP_EST) {
        for (int it = 0; it < this->m_PointClouds.size() - 1; ++it) {
            size_t src = it;
            size_t dst = it + 1;

            pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
            est.setInputSource(this->m_PointClouds[src]);
            est.setInputTarget(this->m_PointClouds[dst]);

            pcl::CorrespondencesPtr corresp(new pcl::Correspondences);
            est.determineCorrespondences(*corresp);
            lum.setCorrespondences(src, dst, corresp);
        }

        // Close the loop.
        size_t src = this->m_PointClouds.size() - 1;
        size_t dst = 0;

        pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
        est.setInputSource(this->m_PointClouds[src]);
        est.setInputTarget(this->m_PointClouds[dst]);

        pcl::CorrespondencesPtr corresp(new pcl::Correspondences);
        est.determineCorrespondences(*corresp);
        lum.setCorrespondences(src, dst, corresp);
    } else {
        assert(false);
    }

    timer.record();
    timer.printTime("Correspondence computation");

    // Set the algorithm variables.
    lum.setMaxIterations(this->LUM_ITER);
    lum.setConvergenceThreshold(this->LUM_CONV_THRESH);

    // Run the LUM algorithm.
    timer.record();
    lum.compute();
    timer.record();
    timer.printTime("LUM compute");

    // Copy the new poses.
    this->m_Poses.clear();
    for (int it = 0; it < this->m_PointClouds.size(); ++it)
    {
        Eigen::Vector6f eigenPose = lum.getPose(it);

        Pose currPose;
        currPose.x = eigenPose(0);
        currPose.y = eigenPose(1);
        currPose.z = eigenPose(2);
        currPose.roll = eigenPose(3);
        currPose.pitch = eigenPose(4);
        currPose.yaw = eigenPose(5);
        this->m_Poses.push_back(currPose);
    }

    // Save point cloud pointer.
    this->m_PcResult = lum.getConcatenatedCloud();
}

void PclReader::printPc(const string &filePath) {
    cout << "Printing complete point cloud to " << filePath << "..." << endl;

    // Open stream to file.
    ofstream file(filePath.c_str());

    for (pcl::PointCloud<pcl::PointXYZ>::iterator it = this->m_PcResult->begin();
         it != this->m_PcResult->end(); ++ it)
    {
        pcl::PointXYZ pt = *it;
        file << pt.x << " " << pt.y << " " << pt.z << endl;
    }

    file.close();
}
