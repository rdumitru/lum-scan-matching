//==============================================================================
// Includes.
//==============================================================================
// User includes.
#include <reader/PclReader.h>
#include <timer/Timer.h>

// C++ includes.
#include <iostream>
using namespace std;

// Pcl includes.
#include <pcl/console/parse.h>
#include <pcl/console/print.h>

//==============================================================================
// Main.
//==============================================================================
int main(int argc, char* argv[]) {
    Timer timer;
    timer.start();

    // Parse arguments.
    string path = "/media/Mobile/Scans/lum";
    pcl::console::parse_argument(argc, argv, "-p", path);
    cout << "Path: " << path << "..." << endl;

    int start = 0;
    pcl::console::parse_argument(argc, argv, "-s", start);
    cout << "Start: " << start << "..." << endl;

    int end = 0;
    pcl::console::parse_argument(argc, argv, "-e", end);
    cout << "End: " << end << "..." << endl;

    // Read and process the point cloud.
    PclReader reader(CORRESP_EST);
    reader.read(path, 0, 3);
    reader.run();

    vector<Pose> poses = reader.getPoses();
    for (vector<Pose>::iterator it = poses.begin();
         it != poses.end(); ++it)
    {
        Pose ps = *it;
        cout << ps.x << " " << ps.y << " " << ps.z << endl;
        cout << ps.roll << " " << ps.pitch << " " << ps.yaw << endl;
    }

    reader.printPc("/home/rdumitru/Downloads/scan000.3d");

    timer.record();
    timer.printTime("Total time");

    cout << "Program end..." << endl;
}

