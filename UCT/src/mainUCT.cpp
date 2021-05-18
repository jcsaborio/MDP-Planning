/*
 * Launcher for the VI algorithm using the Maze problem.
 * 
 * by Juan Carlos Saborio, DFKI Labor Niedersachsen (2021).
 */
#include <iostream>
#include <cstring>
#include "maze.h"
#include "UCT.h"
#include "ParserUCT.h"

using std::cout;
using std::endl;

int main(int argc, char ** argv){
    PARAMS mazeParams;
    UCT_PARAMS uctParams;
    EXP_PARAMS expParams;
    PARSER::COMMAND_LINE cl;
            
    if(argc >= 3){
        PARSER::parseCommandLine(argv, argc, cl);
    }
    else{
        std::cerr << "Must at least specify input file." << endl;
        return -1;
    }
    
    if(!PARSER::parseMaze(mazeParams, uctParams, cl.inputFile)){
        std::cerr << "Could not parse problem file." << endl;
        return -1;
    }
    
    expParams.minSims = cl.minSims;
    expParams.maxSims = cl.maxSims;
    expParams.numRuns = cl.runs;
    expParams.numSteps = cl.numSteps;
    expParams.outputFile = cl.outputFile;
    
    //Create maze with parameters
    Maze * M = new Maze(mazeParams);    
    cout << "Maze: " << endl;
    M->DisplayState(*uctParams.startstate, cout);
    
    //Create UCT with parameters
    UCT uct(uctParams, expParams, M);
    
    uct.Experiment();
    
    
    return 0;
}
