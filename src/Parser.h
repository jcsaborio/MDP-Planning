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
#include "vi.h"

using std::cout;
using std::endl;
using std::string;

namespace PARSER{
           
    bool parseMaze(PARAMS& mazeParams, VI_PARAMS& viParams, char* inputFile){
        std::ifstream infile(inputFile);

        if(!infile.is_open()){
            cout << "Could not open file \"" << inputFile << "\"." << endl;
            return false;
        }
        
        string param, s_value;
        int goalC = 0;
        int goalR = 0;
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
                viParams.discount = stof(s_value);
            else if(param == "error")
                viParams.error = stof(s_value);
            else if(param == "goalC")
                goalC = stoi(s_value);
            else if(param == "goalR")
                goalR = stoi(s_value);
            else
                cout << "\tWarning: \"" << param << "\" is not a valid parameter." << endl;
        }
        
        infile.close();
        
        //If not set, goal is bottom right corner
        if(goalC == 0) goalC = mazeParams.cols - 1;
        if(goalR == 0) goalR = mazeParams.rows - 1;
        
        mazeParams.goal = new State(goalC, goalR);
        
        cout << "Parsed Maze: " << mazeParams.rows << "x" << mazeParams.cols << ", " 
             << mazeParams.traps << " traps, " << "p(traps) = " << mazeParams.p_traps 
             << ", gamma = " << viParams.discount << ", error = " << viParams.error << endl;
        
        return true;
    }
};

#endif
