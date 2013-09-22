//==============================================================================
// Includes.
//==============================================================================
// C++ includes.
#include <iostream>
#include <fstream>
using namespace std;

// PCL includes.
#include <pcl/console/parse.h>

//==============================================================================
// Main.
//==============================================================================
int main(int argc, char* argv[]) {
    string inPath = "/media/Mobile/Scans/lum/scan000.3d";
    pcl::console::parse_argument(argc, argv, "-i", inPath);

    string outPath = "/media/Mobile/Scans/lum/scan000.3d.trans";
    pcl::console::parse_argument(argc, argv, "-o", outPath);

    double trans = 10.0;
    pcl::console::parse_argument(argc, argv, "-t", trans);

    // Read in file.
    ifstream in(inPath.c_str());
    ofstream out(outPath.c_str());

    double x, y, z;
    while (!in.eof()) {
        in >> x >> y >> z;

        x += trans;
        y += trans;
        z += trans;

        out << x << " " << y << " " << z << endl;
    }

    in.close();
    out.close();

    return 0;
}
