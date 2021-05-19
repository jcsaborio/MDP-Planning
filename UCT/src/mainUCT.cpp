/*
 * Launcher for UCT using the Maze problem.
 * 
 * by Juan Carlos Saborio, DFKI Labor Niedersachsen (2021).
 * 
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
           
    PARSER::parseCommandLine(argv, argc, cl);
    
    if(!PARSER::parseMaze(mazeParams, uctParams, cl.inputFile)){
        std::cerr << "Could not parse problem file." << endl;
        return -1;
    }
    
    //Assign values parsed from command line
    expParams.minSims = cl.minSims;
    expParams.maxSims = cl.maxSims;
    expParams.numRuns = cl.runs;
    expParams.numSteps = cl.numSteps;
    expParams.outputFile = cl.outputFile;
    expParams.verbose = cl.verbose;
    
    //Create maze
    Maze * M = new Maze(mazeParams);    
    cout << "Maze: " << endl;
    M->DisplayState(*uctParams.startstate, cout);
    
    //Create UCT (planner)
    UCT uct(uctParams, expParams, M);
    
    /* Run UCT with specified parameters
     * Solve() generates and prints a deterministic policy (not useful in larger problems)
     * Experiment() runs UCT online several times following the conditions in expParameters, and generates an output file.
     */
    if(cl.solve)
        uct.Solve();
    else
        uct.Experiment();
    
    delete M;
    
    return 0;
}
