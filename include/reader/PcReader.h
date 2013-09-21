#ifndef _PC_READER_H
#define _PC_READER_H


//==============================================================================
// Includes.
//==============================================================================
// C++ includes.
#include <string>
#include <vector>

//==============================================================================
// Helpers.
//==============================================================================
struct Pose {
    double x, y, z;
    double roll, pitch, yaw;
};

//==============================================================================
// Class declaration.
//==============================================================================
class PcReader {
protected:
    std::vector<Pose> m_Poses;

public:
    // Constants.
    static const char fileSep = '/';

    // Constructors.
    PcReader();

    PcReader(const PcReader &other);

    virtual ~PcReader();

    // Class members.
    virtual void read(const std::string &path,
              const int &start = 0, const int &end = 0, const int &width = 3,
              const std::string &root = "scan", const std::string &ext = ".3d", const std::string &poseExt = ".pose") = 0;

    virtual void run() = 0;

    // Getters and setters.
    std::vector<Pose> getPoses();
};

#endif // _PC_READER_H
