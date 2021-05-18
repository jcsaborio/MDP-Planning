/* UCT
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
#include "maze.h"

using std::vector;
using std::cout;
using std::endl;

class Node{
    private:
        vector<int> actions;
        vector<int> actionCount;
        vector< vector<Node*> *> successors;
        vector<double> reward;
        int count;
        State* s;

    public:
        Node(State* s, vector<int> actions);
        ~Node();
        
        int getAction(int a);
        int getCount();
        void increaseCount();
        void increaseActionCount(int a);
        int getActionCount(int a);
        
        double getValue(int a);
        void addReward(int a, double r);
        
        State& getState();
        //vector<Node*> getSuccessors();
        void setSuccessors(vector< vector<Node*> *> succesors);
        Node* getSuccessor(int action, State& s);
};

struct UCT_PARAMS{    
    //Search params
    int discount;
    double exploration;
    int depth;
    State* startstate;
    State* goalstate;
};

struct EXP_PARAMS{
    //Experiment params
    int minSims;
    int maxSims;
    int sims;
    int numSteps;
    int numRuns;
    std::string outputFile;
};

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
        Node * Root;
        UCT_PARAMS searchParams;
        EXP_PARAMS expParams;
        RESULTS results;
        Maze * MDP;
        
        //vector< vector<Node*> *> expandNode(Node * n);
        void expandNode(Node * n);
    
    public:
        UCT(UCT_PARAMS& searchParams, EXP_PARAMS& expParams, Maze * maze);
        ~UCT();
        
        int Search(Node * n, int nsims);
        int UCB(Node * n, bool greedy = false);        
        double Simulate(State& s, Node * n, int depth);
        double Rollout(State& s, int depth);
        
        void Run();
        void MultiRun();
        void Experiment();
};

#endif
