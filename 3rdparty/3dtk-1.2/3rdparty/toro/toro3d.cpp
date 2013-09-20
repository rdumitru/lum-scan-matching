/**********************************************************************
 *
 * This source code is part of the Tree-based Network Optimizer (TORO)
 *
 *   TORO Copyright (c) 2007 Giorgio Grisetti, Cyrill Stachniss, 
 *                           Slawomir Grzonka and  Wolfram Burgard
 *
 * You are free:
 *   - to Share - to copy, distribute and transmit the work
 *   - to Remix - to adapt the work
 *
 * Under the following conditions:
 *
 *   - Attribution. You must attribute the work in the manner specified
 *     by the author or licensor (but not in any way that suggests that
 *     they endorse you or your use of the work).
 *  
 *   - Noncommercial. You may not use this work for commercial purposes.
 *  
 *   - Share Alike. If you alter, transform, or build upon this work,
 *     you may distribute the resulting work only under the same or
 *     similar license to this one.
 *
 * Any of the above conditions can be waived if you get permission
 * from the copyright holder.  Nothing in this license impairs or
 * restricts the author's moral rights.
 *
 * TORO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  
 **********************************************************************/

#include <sys/time.h>
#include <iostream>
#include <fstream>
#include "treeoptimizer3.hh"

using namespace std;

using namespace AISNavigation;
const char*  message[] =
{
  "*******************************************************************",
  "*                             TORO v 0.1                          *",
  "*              (c) Giorgio Grisetti, Cyrill Stachniss,            *",
  "*                  Slawomir Grzonka, and Wolfram Burgard          *",
  "*******************************************************************",
  "",
  " Usage toro [options] <graph file>",
  " options ",
  " -vl <int>  set verbosity level",
  " -dr        disables node reduction",
  " -mst       use the minimum spanning tree",
  " -ic        enable index renaming (saves memory)",
  " -nib       disable initialization according to observations",
  " -i <int>   perform <int> iterations",
  " -df        dump gnuplot files of the intermediate results",
  " -nde       disable dump error on screen (saves time)",
  " -ar        adaptive restart",
  " -oc        overrides covariances",
  " -ip        ignores preconditioner",
  " -2d        load a 2d file",
  " -sl        sort the constraints according to the length of the path",
  "Enjoy!",
  0
};


std::string stripExtension(const std::string s){
  int i;
  for (i=s.length()-1; i>0; i--){
    string k=s.substr(i,1);
    if (k==".")
      break;
  }
  return s.substr(0,i);
}

std::string getExtension(const std::string s){
  int i;
  for (i=s.length()-1; i>0; i--){
    string k=s.substr(i,1);
    if (k==".")
      break;
  }
  return s.substr(i,s.length()-i);
}

TreeOptimizer3 pg;
bool compressIndices=false;
bool reduceNodes=true;
bool initializeOnTree=true;
int  treeType=0;
int  iterations=100;
bool dumpIterations=false;
bool dumpError=true;
bool adaptiveRestart=false;
int  verboseLevel=0;
bool ignorePreconditioner=false;
bool overrideCovariances=false;
bool twoDimensions = false;

TreeOptimizer3::EdgeCompareMode compareMode=EVComparator<TreeOptimizer3::Edge*>::CompareLevel;
std::string filename;

void printBannerAndStatus(ostream& os){
  os << "#******************************************************************" << endl;
  os << "#                              TORO3D v 0.1                       *" << endl;
  os << "#    (c) G.Grisetti, C.Stachniss, S.Grzonka, and W. Burgard       *" << endl;
  os << "#******************************************************************" << endl;
  os << "#Verbosity Level              = " << verboseLevel << endl;
  os << "#Node Reduction               = " << ((reduceNodes)?"enabled":"disabled") << endl;
  os << "#Tree Construction            = " << ((treeType)?"MST":"Simple") << endl;
  os << "#IndexCompression             = " << ((compressIndices)?"enabled":"disabled") << endl;
  os << "#InitialBoost                 = " << ((initializeOnTree)?"enabled":"disabled") << endl;
  os << "#Dumping Iterations           = " << ((dumpIterations)?"enabled":"disabled") << endl;
  os << "#Iterations                   = " << iterations << endl;
  os << "#Dumping Intermediate Files   = " << ((dumpIterations)?"enabled":"disabled") << endl;
  os << "#Dumping Error                = " << ((dumpError)?"enabled":"disabled") << endl;
  os << "#AdaptiveRestart              = " << ((adaptiveRestart)?"enabled":"disabled") << endl;
  os << "#IgnorePreconditioner         = " << ((ignorePreconditioner)?"enabled":"disabled") << endl;
  os << "#OverrideCovariances          = " << ((overrideCovariances)?"enabled":"disabled") << endl;
  os << "#2D File                      = " << ((twoDimensions)?"enabled":"disabled") << endl;
  os << "#Edge Sorting                 = " << ((compareMode==EVComparator<TreeOptimizer3::Edge*>::CompareLevel)?"level":"length") << endl;
  os << "#***********************************" << endl;
  os << endl;
}

int main (int argc, const char** argv){

  if (argc==1){
    int i=0;
    while (message[i]!=0){
      cerr << message[i++] << endl;
    }
    return 0;
  }
  int c=1;
  while (c<argc){
    if (string(argv[c])=="-vl"){
      verboseLevel=atoi(argv[++c]);
    } else if (string(argv[c])=="-dr"){
      reduceNodes=false;
    } else if (string(argv[c])=="-mst"){
      treeType=1;
    } else if (string(argv[c])=="-st"){
      treeType=0;
    } else if (string(argv[c])=="-ic"){
      compressIndices=true;
    } else if (string(argv[c])=="-nib"){
      initializeOnTree=false;
    } else if (string(argv[c])=="-i"){
      iterations=atoi(argv[++c]);
    } else if (string(argv[c])=="-df"){
      dumpIterations=true;
    } else if (string(argv[c])=="-nde"){
      dumpError=false;
    } else if (string(argv[c])=="-ar"){
      adaptiveRestart=true;
    } else if (string(argv[c])=="-ip"){
      ignorePreconditioner=true;
    } else if (string(argv[c])=="-oc"){
      overrideCovariances=true;
    } else if (string(argv[c])=="-2d"){
      twoDimensions=true;
    } else if (string(argv[c])=="-sl"){
      compareMode=EVComparator<TreeOptimizer3::Edge*>::CompareLevel;
    } else {
      filename=argv[c];
      break;
    }
    c++;
  }

  if (filename == "") {
    cerr << "FATAL ERROR: Specify a graph file to load. Aborting." << endl;
    return 0;
  }
    
  printBannerAndStatus(cerr);

  pg.verboseLevel=verboseLevel;

  cerr << "Loading graph file \"" << filename << "\"... ";
  if (!pg.load( filename.c_str(), overrideCovariances, twoDimensions)) {
    cerr << "FATAL ERROR: Could not read file. Abrting." << endl;
    return 0;
  }
  cerr << "Done" << endl;
  
  cerr << " #nodes:" << pg.vertices.size() << " #edges:" << pg.edges.size() << endl; 

  if (reduceNodes){
    cerr << "Loading equivalence constraints and collapsing nodes... ";
    pg.loadEquivalences(filename.c_str());
    cerr << "Done" << endl; 
    cerr << " #nodes:" << pg.vertices.size() << " #edges:" << pg.edges.size() << endl; 
  }

  pg.restartOnDivergence=adaptiveRestart;

  if (compressIndices){
    cerr << "Compressing indices... ";
    pg.compressIndices();
    cerr << "Done" << endl;
  }

  switch (treeType){
  case 0:
    cerr << "Incremental tree construction... ";
    pg.buildSimpleTree();
    cerr << "Done" << endl;
    break;
  case 1:
    cerr << "MST construction... ";
    pg.buildMST(pg.vertices.begin()->first);
    cerr << "Done" << endl;
    break;
  default:
    cerr << " FATAL ERROR: Invalid tree type. Aborting!";
    return -1;
  }
  
  if (initializeOnTree){
    cerr << "Computing initial guess from observations... ";
    pg.initializeOnTree();
    cerr << "Done" << endl;
  }

  cerr << "Initializing the optimizer... ";
  pg.initializeTreeParameters();
  pg.initializeOptimization(compareMode);
  double l=pg.totalPathLength();
  int nEdges=pg.edges.size();
  double apl=l/(double)(nEdges);
  cerr << "Done" << endl;

  cerr << " Average path length=" << apl << endl;
  cerr << " Complexity of an iteration=" << l  << endl;

  string strippedFilename=stripExtension(filename);
  string extension=getExtension(filename);

  cerr << "Saving starting graph... ";
  string output=strippedFilename+"-treeopt-initial.graph";
  pg.save(output.c_str());
  cerr << "Done" << endl << endl;
  output=strippedFilename+"-treeopt-initial.dat";
  pg.saveGnuplot(output.c_str());
  cerr << "Done" << endl << endl;
  string errorOutput=strippedFilename+"-treeopt-error.dat";
  ofstream errorStream;


  if (dumpError){
    errorStream.open(errorOutput.c_str());
    printBannerAndStatus(errorStream);
    errorStream << "# InputFile : " << filename << endl;
    errorStream << "# Nodes : " << pg.vertices.size() << " #edges: " << pg.edges.size() << endl; 
    errorStream << "# Average path length: " << apl << endl;
    errorStream << "# Complexity of an iteration: " << l  << endl;

    errorStream << "# Line Format:" << endl
		<< "#   1:iteration" << endl
		<< "#   2:RotationalGain" << endl
		<< "#   3:error" << endl
		<< "#   4:error/constraint" << endl
		<< "#   5:max rotational error" << endl
		<< "#   6:average rotational error" << endl
		<< "#   7:max translational error" << endl
		<< "#   8:average translational error" << endl;
  }



  bool corrupted=false;
  cerr << "**** Starting optimization ****" << endl;
  struct timeval ts, te;
  gettimeofday(&ts,0);
  for (int i=0; i<iterations; i++){
    pg.iterate(0,ignorePreconditioner);
    if (dumpIterations){
      char b[10];
      sprintf(b,"%04d",i);
      string output=strippedFilename+"-treeopt-" + b + ".dat";
      pg.saveGnuplot(output.c_str());
    }
    if (dumpError){
      // compute the error and dump it
      double mte, mre, are, ate;
      double error=pg.error(&mre, &mte, &are, &ate);
      cerr << "iteration " << i << " RotGain=" << pg.getRotGain() << endl
           << "  global error = " << error << "   error/constraint = " << error/nEdges << endl;
      cerr << "  mte=" << mte << "  mre=" << mre << " are=" << are << " ate=" << ate << endl;
      errorStream << i << " " 
		  << pg.getRotGain() << " "
		  << error << " "
		  << error/nEdges << " "
		  << mre << " "
		  << are << " "
		  << mte << " "
		  << ate << endl;

      cout << error << endl;
      if (mre>(M_PI/2)*(M_PI/2))
	corrupted=true;
      else
	corrupted=false;
   }
  }
  gettimeofday(&te,0);
  cerr << "**** Optimization Done ****" << endl;

  double dts=(te.tv_sec-ts.tv_sec)+1e-6*(te.tv_usec-ts.tv_usec);
  cerr << "TOTAL TIME= " << dts << " s." << endl;
  
  if (corrupted)
    strippedFilename=strippedFilename+"-corrupted";
  cerr << "Saving files...(graph file)" << endl;
  output=strippedFilename+"-treeopt-final.graph";
  pg.save(output.c_str());
  cerr << "...(gnuplot file)..." << endl;
  output=strippedFilename+"-treeopt-final.dat";
  pg.saveGnuplot(output.c_str());
  errorStream.close();
  cerr << "Done" << endl;

}
