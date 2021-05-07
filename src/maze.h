/*
 * Maze:
 * by Juan Carlos Saborio, DFKI Labor Niedersachsen (2021)
 * This is a simple maze navigation problem implemented as an MDP.
 * 
 * - The step function simulates the transition s' <- (s,a) and returns the associated reward R(s,a,s') and probability.  This function may be used with sampling-based planning, e.g. UCT.
 * - The expandMDP function returns all resulting states, rewards and probabilities for a given pair (s,a) and may be used for full-width planners such as VI and PI.
 */


#ifndef MAZE_H
#define MAZE_H

#include <iostream>
#include <ostream>
#include <vector>
#include <cstdlib>
#include <ctime>

using std::vector;

/*
 * The state contains only the agent's location.  The grid is persistent so it is part of the maze itself.
 */
struct State{
    int row;
    int col;
    
    State(int r, int c){ row = r; col = c; }
    bool equals(State s2){ return (row == s2.row && col == s2.col); }
    bool equals(int r, int c) const{ return (row == r && col == c); };
};

/*
 * Maze parameters
 */
struct PARAMS{                   
    int cols; //No. of columns
    int rows; //No. of rows
    int traps; //No. of traps
    float p_traps = 0.5; //Probability of getting trapped
    State* goal; //Location of the goal
};

/*
 * MDP definition
 */
class Maze{
    private:
        int cols, rows; //Grid size
        int traps; //No. of traps
        float p_traps; //Prob. of getting trapped
        float discount; //Discount factor
        State* goalstate; //Location of the goal
        char ** grid;
        void InitMaze();
        bool Bernoulli(double p) const; //Simulate the outcome of a Bernoulli trial with probability p
        
    public:
        Maze(PARAMS& mazeParams);
        bool Step(State& s, int action, double& reward) const; //Step function for generative planning
        
        /*
         * These functions are used for full-width planning (e.g. VI/PI).
         */
        void listStates(vector<State>& states) const; //List all states in this MDP
        void expandMDP(const State& origin, int action, vector<State>& nextStatesV, vector<double>& rewardV, vector<float>& probabilityV) const; //Generate all successors, rewards and their probabilities for a given state-action pair
        
        /*
         * Utility functions
         */
        int getRows() const { return rows; }
        int getCols() const { return cols; }
        int getNumStates() const { return rows*cols; }
        int getNumActions() const { return nActions; }
        char ** getGrid(){ return grid; }
        bool validateState(const State& s) const { return (s.row >=0 && s.row < rows && s.col >= 0 && s.col < cols); }
        
        void getActions(State& s, vector<int>& actions) const; //Get all actions available in state s
        
        /*
         * Output
         */
        void DisplayState(const State& state, std::ostream& ostr) const;
        void DisplayAction(int action, std::ostream& ostr) const;
        
    
    protected:        
        /*
         * Reward distribution (change if desired):
         */
        int rStep = -1;
        int rOut = -10;
        int rTrap = -5;
        int rGoal = 10;
        
        //Actions:
        int nActions = 4;
        enum{
            UP,
            DOWN,
            LEFT,
            RIGHT
        };
        
        //Display characters
        char tile = '_';
        char agent = '*';
        char trap = 'O';
        char goal = 'X';              

};

inline std::ostream& operator<<(std::ostream& ostr, State& s){
    ostr << "(" << s.row << ", " << s.col << ")";
    return ostr;
}

inline std::ostream& operator<<(std::ostream& ostr, Maze& maze){
        int rows = maze.getRows();
        int cols = maze.getCols();
        char ** grid = maze.getGrid();
        
        for(int i=0; i < rows; i++){
            for(int j=0; j < cols; j++){
                ostr << " " << grid[i][j] << " ";
            }
            std::cout << std::endl;
        }
        
    return ostr;
}


#endif
