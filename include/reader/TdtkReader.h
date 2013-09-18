//==============================================================================
// Includes.
//==============================================================================
#include <reader/PcReader.h>

//==============================================================================
// Class declaration.
//==============================================================================
class TdtkReader : public PcReader
{
public:
    // Constructors.
    TdtkReader();

    TdtkReader(const TdtkReader &other);

    ~TdtkReader();

    // Class members.
    void read(const std::string &path,
              const int &start = 0, const int &end = 0, const int &width = 3,
              const std::string &root = "scan", const std::string &ext = ".3d", const std::string &poseExt = ".pose");

    void run();
};
