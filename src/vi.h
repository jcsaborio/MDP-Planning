/*
    VI:
    by Juan Carlos Saborio, DFKI Labor Niedersachsen (2021)
    
    VI implements a straightforward version of the value iteration algorithm.
 */

#ifndef VI_H
#define VI_H

#define Infinity 1e+10

#include <vector>
#include <iostream>
#include <ostream>
#include <cassert>
#include <algorithm>
#include <cmath>
#include "maze.h"

using std::vector;
using std::cout;
using std::endl;

/*
    Planning parameters
 */
struct VI_PARAMS{
    //State* startstate;  //Not generally used in VI
    float discount; //Discount factor for expected returns
    double error; //Convergence criteria
};

class VI{
    private:
        VI_PARAMS PlanParams;
        double * V; //Array for state values        
        Maze * maze; //The planning domain
        int numStates;
        int numActions;
        
        int arg_max(vector<double> values); //Return the action with maximal value
        double max(vector<double> values); //Return a maximal value
        double getValue(const State& s); //Return the current value of state s
        void setValue(const State& s, double v); //Set the value of state s to v
        
    public:
        VI(VI_PARAMS& PlanParams, Maze * maze);
        void Plan(); //VI using the error in VI_PARAMS
        void Plan(double error); //VI using given error
        void DisplayPolicy(); //Print current optimal policy to stdout
        void DisplayPolicy(std::ostream& ostr); //Display optimal policy
};

#endif
