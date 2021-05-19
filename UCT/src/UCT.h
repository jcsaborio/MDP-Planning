/* UCT
 * 
 * Simple UCT implementation
 * by Juan Carlos Saborio,
 * DFKI Labor Niedersachsen (2021)
 * 
 */

#ifndef UCT_H
#define UCT_H

#define Infinity 1e+10
#define DiscountDepth -4.6052 //log(0.01)

#include <cassert>
#include <vector>
#include <cmath>
#include <cstring>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include "maze.h"

using std::vector;
using std::cout;
using std::endl;

/*
 * Node defines both the contents of the MCTS tree nodes as well as the tree structure itself
 */
class Node{
    private:
        vector<int> actions; //List of actions
        vector<int> actionCount; //No. of times each action has been executed
        vector< vector<Node*> *> successors; //Somewhat convoluted way to maintain a successor map/matrix.  Each action has multiple successors.
        vector<double> reward; //List of rewards for each action
        int count; //Times the node has been visited
        State* s; //The MDP state in this tree node

    public:
        Node(const State& s, vector<int> actions);
        ~Node();
        
        int getAction(int a);
        int getCount();
        void increaseCount();
        void increaseActionCount(int a);
        int getActionCount(int a);
        
        double getValue(int a); //Compute Q(s,a)
        void addReward(int a, double r); //Update the sum of rewards to compute Q
        
        State& getState();
        vector< vector<Node*> *> * getSuccessorsVector();
        void setSuccessors(vector< vector<Node*> *> succesors);
        Node* getSuccessor(int action, State& s);
        void freeSuccessor(int action, State& s);
        bool expanded();
};

//Search params
struct UCT_PARAMS{
    double discount;
    double exploration = 20;
    int depth;
    State* startstate;
    State* goalstate;
};

//Experiment params
struct EXP_PARAMS{
    int minSims;
    int maxSims;
    int sims;
    int numSteps;
    int numRuns;
    std::string outputFile;
    int verbose = 1;
};

//Store experiment results
struct RESULTS{
    vector<double> time;
    vector<double> reward;
    vector<double> undiscountedReturn;
    vector<double> discountedReturn;
    
    void clear();
};

inline void RESULTS::clear()
{
    time.clear();
    reward.clear();
    discountedReturn.clear();
    undiscountedReturn.clear();	 
}

class UCT{
    private:
        Node * Root; //The root of the MCTS tree
        UCT_PARAMS searchParams;
        EXP_PARAMS expParams;
        RESULTS results;
        Maze * MDP; //The planning domain
                
        void expandNode(Node * n); //Create node successors
    
    public:
        UCT(UCT_PARAMS& searchParams, EXP_PARAMS& expParams, Maze * maze);
        //~UCT();
        
        int Search(Node * n, int nsims); //Plan with UCT from node n, using nsims simulations
        int UCB(Node * n, bool greedy = false); //UCB action selection
        double Simulate(State& s, Node * n, int depth); //MCTS simulation
        double Rollout(State& s, int depth); //MCTS Rollout
        
        /*
         * Execution and testing functions
         */
        void Run(); //Run a single instance according to expParams
        void MultiRun(); //Run several instances according to expParams
        void Experiment(); //Coordinate and generate output for multiple instances according to expParams
        
        /*
         * Generate complete policy by iterating over all states
         */
        void Solve();
};

#endif
