//==============================================================================
// Includes.
//==============================================================================
#include <reader/TdtkReader.h>

#define MAX_OPENMP_NUM_THREADS  8
#define OPENMP_NUM_THREADS      8

#include "slam6d/icp6Dlumeuler.h"
#include "slam6d/icp6Dlumquat.h"
#include "slam6d/icp6Dminimizer.h"

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
void TdtkReader::read(const std::string &path,
                     const int &start, const int &end, const int &width,
                     const std::string &root, const std::string &ext, const std::string &poseExt)
{}

void TdtkReader::run()
{
    icp6Dminimizer *icp6Dminimizer = new icp6D_LUMEULER(true);



}
