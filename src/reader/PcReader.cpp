//==============================================================================
// Includes.
//==============================================================================
#include <reader/PcReader.h>

// User includes.
#include <common.h>

// C++ includes.
#include <string>
using namespace std;

// C includes.
#include <assert.h>

//==============================================================================
// Class implementation.
//==============================================================================

// Constructors.
PcReader::PcReader()
{
    this->m_Poses.clear();
}

PcReader::PcReader(const PcReader &other)
{
    this->m_Poses = other.m_Poses;
}

PcReader::~PcReader()
{}

vector<Pose> PcReader::getPoses()
{
    return this->m_Poses;
}
