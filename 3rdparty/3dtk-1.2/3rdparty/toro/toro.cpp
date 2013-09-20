/**********************************************************************
 *
 * This source code is part of the Tree-based Network Optimizer (TORO)
 *
 *   TORO Copyright (c) 2007 Giorgio Grisetti, Cyrill Stachniss, 
 *                           Slawomir Grzonka and  Wolfram Burgard
 *
 * TORO is licences under the Common Creative License,
 * Attribution-NonCommercial-ShareAlike 3.0
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
#include <stdio.h>
#include "treeoptimizer2.hh"
#include <sys/time.h>


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
  " -df        dump gnuplot files of the intermediateresults",
  " -nde       disable dump error on screen (saves time)",
  " -oc        override covariances from the file (deals with corrupted input)",
  "",
  "Enjoy!",
  0
};

string stripExtension(const string s){
  int i;
  for (i=s.length()-1; i>0; i--){
    string k=s.substr(i,1);
    if (k==".")
      break;
  }
  return s.substr(0,i);
}

string getExtension(const string s){
  int i;
  for (i=s.length()-1; i>0; i--){
    string k=s.substr(i,1);
    if (k==".")
      break;
  }
  return s.substr(i,s.length()-i);
}

int main (int argc, const char** argv){
  TreeOptimizer2 pg;
  bool compressIndices=false;
  bool reduceNodes=true;
  bool initializeOnTree=true;
  int  treeType=0;
  int  iterations=100;
  bool dumpIterations=false;
  bool dumpError=true;
  bool overrideCovariances=false;
  int  verboseLevel=0;

  string filename;

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
    } else if (string(argv[c])=="-oc"){
      overrideCovariances=true;
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
    


  cerr << "*******************************************************************" << endl;
  cerr << "*                              TORO v 0.1                         *" << endl;
  cerr << "*              (c) Giorgio Grisetti, Cyrill Stachniss,            *" << endl;
  cerr << "*                  Slawomir Grzonka and Wolfram Burgard           *" << endl;
  cerr << "*******************************************************************" << endl;
  cerr << " Verbosity Level              = " << verboseLevel << endl;
  cerr << " Node Reduction               = " << ((reduceNodes)?"enabled":"disabled") << endl;
  cerr << " Tree Construction            = " << ((treeType)?"MST":"Simple") << endl;
  cerr << " IndexCompression             = " << ((compressIndices)?"enabled":"disabled") << endl;
  cerr << " InitialBoost                 = " << ((initializeOnTree)?"enabled":"disabled") << endl;
  cerr << " Dumping Iterations           = " << ((dumpIterations)?"enabled":"disabled") << endl;
  cerr << " Iterations                   = " << iterations << endl;
  cerr << " Dumping Intermediate Files   = " << ((dumpIterations)?"enabled":"disabled") << endl;
  cerr << " Dumping Error                = " << ((dumpError)?"enabled":"disabled") << endl;
  cerr << " Override Covariances         = " << ((overrideCovariances)?"enabled":"disabled") << endl;
  cerr << "************************************" << endl;
  cerr << endl;

  pg.verboseLevel=verboseLevel;

  cerr << "Loading graph file... ";
  if (!pg.load( filename.c_str(), overrideCovariances)) {
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
  pg.initializeOptimization();
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

  cerr << "Saving starting graph (gnuplot_... ";
  output=strippedFilename+"-treeopt-initial.dat";
  pg.saveGnuplot(output.c_str());
  cerr << "Done" << endl << endl;


  cerr << "**** Starting optimization ****" << endl;
  struct timeval ts, te;
  gettimeofday(&ts,0);
  for (int i=0; i<iterations; i++){
    pg.iterate();
    if (dumpIterations){
      char b[10];
      sprintf(b,"%04d",i);
      string output=strippedFilename+"-treeopt-" + b + ".dat";
      pg.saveGnuplot(output.c_str());
    }
    if (dumpError){
      // compute the error and dump it
      double error=pg.error();
      cerr << "iteration = " << i << "  global error = " << error << "   error/constraint = " << error/nEdges << endl;
    }
  }
  gettimeofday(&te,0);
  cerr << "**** Optimization Done ****" << endl;

  double dts=(te.tv_sec-ts.tv_sec)+1e-6*(te.tv_usec-ts.tv_usec);
  cerr << "TOTAL TIME= " << dts << " s." << endl;

  cerr << "Saving files...(graph file)" << endl;
  output=strippedFilename+"-treeopt-final.graph";
  pg.save(output.c_str());
  cerr << "...(gnuplot file)..." << endl;
  output=strippedFilename+"-treeopt-final.dat";
  pg.saveGnuplot(output.c_str());

  cerr << "Done" << endl;
} 
