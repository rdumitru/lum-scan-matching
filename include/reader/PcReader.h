#ifndef _PC_READER_H
#define _PC_READER_H


//==============================================================================
// Includes.
//==============================================================================
// C++ includes.
#include <string>
#include <vector>
#include <ostream>

#include "common.h"

//==============================================================================
// Helpers.
//==============================================================================
struct Pose {
    double x, y, z;
    double roll, pitch, yaw;

    Pose(){}
    Pose(double translation[3], double rotationEulerAngles[3]) :
        x(translation[0]), y(translation[1]), z(translation[2]),
        roll(rotationEulerAngles[0]), pitch(rotationEulerAngles[1]),
        yaw(rotationEulerAngles[2]) {}

    friend std::ostream& operator<<(std::ostream& o, const Pose& pose);
};

inline std::ostream& operator<<(std::ostream& o, const Pose& p){
    o << "T: " << p.x << " " << p.y << " " << p.z
      << " roll: " << rad2Deg(p.roll) << " pitch: " << rad2Deg(p.pitch)
      << " yaw: " << rad2Deg(p.yaw) << "\n";
    return o;
}

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
