//==============================================================================
// Includes.
//==============================================================================
#include <string>

#include <reader/TdtkReader.h>

#define MAX_OPENMP_NUM_THREADS  8
#define OPENMP_NUM_THREADS      8

#include "slam6d/icp6Dlumeuler.h"
#include "slam6d/icp6Dlumquat.h"
#include "slam6d/icp6Dminimizer.h"
#include "slam6d/scan.h"
#include "slam6d/icp6D.h"
#include "slam6d/lum6Deuler.h"

using namespace std;

//==============================================================================
// Class implementation.
//==============================================================================
// Constructors.
TdtkReader::TdtkReader() : PcReader()
{}

TdtkReader::TdtkReader(const TdtkReader &other) : PcReader(other)
{}

TdtkReader::~TdtkReader()
{}

// Methods.
void TdtkReader::read(const std::string& path,
                     const int &start, const int &end, const int &width,
                     const std::string& root, const std::string &ext, const std::string &poseExt)
{
    Scan::readScansRedSearch(UOS, start, end, path, 100000.0, 0,
                             -1.0, 1,
                             simpleKD, false, true);
}

void TdtkReader::run()
{
    const int min_clpairs = 6;
    const int min_loop_size = 10;
    const int num_iterations_icp = 3;
    const int num_iterations_graphslam = 3;

    icp6Dminimizer *icp6Dminimizer = new icp6D_LUMEULER(true);

    icp6D* icpAlgo = new icp6D(icp6Dminimizer, 25.0, num_iterations_icp);
    icpAlgo->doICP(Scan::allScans);

    graphSlam6D *graphSlam6DAlgo = new lum6DEuler(icp6Dminimizer);
    graphSlam6DAlgo->matchGraph6Dautomatic(Scan::allScans,
                                           num_iterations_graphslam,
                                           min_clpairs, min_loop_size);

    vector <Scan*>::iterator it = Scan::allScans.begin();
    while (!Scan::allScans.empty()) {
        Scan* scan = Scan::allScans[0];
        Pose pose(scan->rPos, scan->rPosTheta);
        cout << pose << endl;
        delete scan;
    }
}
