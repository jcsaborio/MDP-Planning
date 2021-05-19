/* 
 * Problem file parser
 * 
 * Adapted from a previous parser.
 * 
 * Juan Carlos Saborio, DFKI Labor Niedersachsen (2021)
 */


#ifndef PARSER_H
#define PARSER_H

#include <fstream>
#include <iostream>
#include <iomanip>
#include "maze.h"
#include "UCT.h"

using std::cout;
using std::endl;
using std::string;

namespace PARSER{
    
    struct COMMAND_LINE{
        string help;        
        string inputFile = "none";
        string outputFile = "output.txt";        
        int minSims = 2;
        int maxSims = 16;
        int numSteps = 50;
        int runs = 1;
        int verbose = 1;
        bool solve = false;
    };
    
    void parseCommandLine(char ** argv, int argc, COMMAND_LINE& cl){        
        string param, value;
        for(int i=1; i<argc; i+=2){
            param = argv[i];
            
            if(argc > i+1) value = argv[i+1];
            if(param == "--help"){
                cout << "UCT algorithm for the Maze problem" << endl;
                cout << "Parameters" << endl;
                
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--inputFile";
                cout << std::left << std::setw(100) << "Problem specification file" << endl;
                
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--outputFile";
                cout << std::left << std::setw(100) << "Summary output file" << endl;
                
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--minSims";
                cout << std::left << std::setw(100) << "Min. amount of simulations (power of 2)" << endl;
                
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--maxSims";
                cout << std::left << std::setw(100) << "Max. amount of simulations (power of 2)" << endl;
                
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--numSteps";
                cout << std::left << std::setw(100) << "Max. steps before termination" << endl;
                
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--runs";
                cout << std::left << std::setw(100) << "No. of episodes" << endl;
                
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--verbose";
                cout << std::left << std::setw(100) << "Verbosity level (default = 1)" << endl;      
                
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--solve";
                cout << std::left << std::setw(100) << "Generate deterministic policy using N simulations per step" << endl;
                
                exit(0);
            }
            
            else if(param == "--inputFile")
                cl.inputFile = value;
            else if(param == "--outputFile")
                cl.outputFile = value;            
            else if(param == "--minSims")
                cl.minSims = stoi(value);
            else if(param == "--maxSims")
                cl.maxSims = stoi(value);
            else if(param == "--numSteps")
                cl.numSteps = stoi(value);
            else if(param == "--runs")
                cl.runs = stoi(value);
            else if(param == "--verbose")
                cl.verbose = stoi(value);
            else if(param == "--solve"){
                cl.maxSims = stoi(value);
                cl.solve = true;
            }
            else
                cout << "Unrecognized parameter \"" << param << "\"" << endl;
        }
        
    }
           
    bool parseMaze(PARAMS& mazeParams, UCT_PARAMS& uctParams, string inputFile){
        std::ifstream infile(inputFile);

        if(!infile.is_open()){
            cout << "Could not open file \"" << inputFile << "\"." << endl;
            return false;
        }
        
        string param, s_value;
        int goalC = 0;
        int goalR = 0;
        int startC = 0;
        int startR = 0;
        while(infile >> param >> s_value){
            //cout << param << " = " << s_value << endl;
            if(param == "cols")
                mazeParams.cols = stoi(s_value);
            else if(param == "rows")
                mazeParams.rows = stoi(s_value);
            else if(param == "traps")
                mazeParams.traps = stoi(s_value);
            else if(param == "p_traps")
                mazeParams.p_traps = stof(s_value);            
            else if(param == "discount")
                uctParams.discount = stof(s_value);            
            else if(param == "goalC")
                goalC = stoi(s_value);
            else if(param == "goalR")
                goalR = stoi(s_value);
            else if(param == "startC")
                startC = stoi(s_value);
            else if(param == "startR")
                startR = stoi(s_value);
            else
                cout << "\tWarning: \"" << param << "\" is not a valid parameter." << endl;
        }
        
        infile.close();
        
        //If not set, goal is bottom right corner
        if(goalC == 0) goalC = mazeParams.cols - 1;
        if(goalR == 0) goalR = mazeParams.rows - 1;
        
        if(startC == 0) startC = 0;
        if(startR == 0) startR = 0;
        
        mazeParams.goal = new State(goalR, goalC);
        uctParams.startstate = new State(startR, startC);
        uctParams.goalstate = new State(goalR, goalC);
        
        cout << "Parsed Maze: " << mazeParams.rows << "x" << mazeParams.cols << ", " 
             << mazeParams.traps << " traps, " << "p(traps) = " << mazeParams.p_traps 
             << ", gamma = " << uctParams.discount << endl;
        
        return true;
    }
};

#endif
