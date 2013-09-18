//==============================================================================
// Includes.
//==============================================================================
#include <reader/TdtkReader.h>

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

// Class members.
void TdtkReader::read(const std::string &path,
                     const int &start, const int &end, const int &width,
                     const std::string &root, const std::string &ext, const std::string &poseExt)
{}

void TdtkReader::run()
{}
