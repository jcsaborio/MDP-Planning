/*
 * Launcher for the VI algorithm using the Maze problem.
 * 
 * by Juan Carlos Saborio, DFKI Labor Niedersachsen (2021).
 */
#include <iostream>
#include <cstring>
#include "maze.h"
#include "vi.h"
#include "Parser.h"

using std::cout;
using std::endl;

int main(int argc, char ** argv){
    PARAMS mazeParams;
    VI_PARAMS viParams;
    
    char * inputFile;
    if(argc >= 2){
        inputFile = argv[1];
    }
    else{
        std::cerr << "Must specify problem file." << endl;
        return -1;
    }
    
    if(!PARSER::parseMaze(mazeParams, viParams, inputFile)){
        std::cerr << "Could not parse problem file." << endl;
        return -1;
    }       
    
    //Create maze with parameters
    Maze * M = new Maze(mazeParams);    
    cout << "Maze: " << endl;
    cout << *M << endl;
    
    //Create VI with parameters
    VI vi(viParams, M);
    //vi.DisplayPolicy(); //Display current policy before value approximation
    
    //Perform value iteration until error criteria is met
    vi.Plan();
    
    //Display the current policy after value approximation
    vi.DisplayPolicy();
    
    return 0;
}
