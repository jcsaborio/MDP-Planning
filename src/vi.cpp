#include "vi.h"

using std::cout;
using std::endl;

VI::VI(VI_PARAMS& params, Maze * maze){
    PlanParams.discount = params.discount;
    PlanParams.error = params.error;
    this->maze = maze;
    numActions = maze->getNumActions();    
    numStates = maze->getRows()*maze->getCols();
    
    V = new double[numStates];
    for(int i=0; i < numStates; i++) V[i] = 0.0;
}

/*
 * Perform value iteration using the error specified as a parameter
 */
void VI::Plan(){
    Plan(PlanParams.error);
}

/*
 * Perform value iteration with given error
 */
void VI::Plan(double error){    
    double previousV;
    double delta = 0.0;    
    vector<double> outcomes;
    double sum_s_p;

    vector<State> states;
    vector<int> actions;    
    vector<State> nextStates;
    vector<double> reward;
    vector<float> probability;
    
    int iter = 0; //Count number of iterations until convergence

    //First get all MDP states
    maze->listStates(states);
    
    do{
        delta = 0.0;
        //Iterate over the entire state space
        for(State& s : states){
            
            //Get the actions available in this state
            maze->getActions(s, actions);
                
            //For every action: get the successor states, their rewards and their probabilities
            for(auto a : actions){
                
                maze->expandMDP(s, a, nextStates, reward, probability);
                
                //Sum over s' of p(s')[r + gamma*V(s')]
                sum_s_p = 0;
                for(int s_p=0; s_p < nextStates.size(); s_p++){
                    sum_s_p += probability[s_p] * (reward[s_p] + PlanParams.discount*getValue(nextStates[s_p]) );
                }
                outcomes.push_back(sum_s_p); //store the sum
                
                //Free memory
                nextStates.clear();
                reward.clear();
                probability.clear();
            }

            //Obtain previous value
            previousV = getValue(s);
            //Now update using the value of the action with the *best* outcome            
            setValue(s, max(outcomes));
            //Update delta using the value difference
            delta = std::max( delta, std::abs(previousV - getValue(s)) );
            
            //Free memory
            outcomes.clear();
            actions.clear();            
        }
        
    iter++; //Count iterations
    
    }while(delta > error); //Stop when no values differ by more than the permitted error
    
    cout << "VI finished after " << iter << " iterations." << endl;
}

double VI::getValue(const State& s){    
    assert(maze->validateState(s));
    return V[s.row * maze->getCols() + s.col];
}

void VI::setValue(const State& s, double v){
    assert(maze->validateState(s));
    V[s.row * maze->getCols() + s.col] = v;
}

/*
 * Return the vector element with maximal value
 */
int VI::arg_max(vector<double> values){
    double best_v = -Infinity;
    double best_p = 0;
    double value = 0;
    
    for(int i=0; i < values.size(); i++){        
        if(values[i] > best_v){
            best_v = values[i];
            best_p = i;
        }
    }
    return best_p;
}

/*
 * Return the maximal value
 */
double VI::max(vector<double> values){
    double best_v = -Infinity;
    double value = 0;
    
    for(auto v : values){
        if(v > best_v){
            best_v = v;
        }
    }
    return best_v;
}

/*
 * Extract the optimal policy computed by value iteration, by simply displaying for each state the action with maximal value
 */
void VI::DisplayPolicy(){
    DisplayPolicy(cout);
}

void VI::DisplayPolicy(std::ostream& ostr){          
    vector<double> outcomes;
    double sum_s_p;
    int best_a;

    vector<State> states;
    vector<int> actions;
    vector<State> nextStates;
    vector<double> reward;
    vector<float> probability;
    
    int cols = maze->getCols();
    
    //Iterate one last time over all states to look for the best valued action
    maze->listStates(states);
    for(State& s : states){            
        maze->getActions(s, actions);
        for(auto a : actions){
            maze->expandMDP(s, a, nextStates, reward, probability);
            sum_s_p = 0;
            for(int s_p=0; s_p < nextStates.size(); s_p++){
                sum_s_p += probability[s_p] * (reward[s_p] + PlanParams.discount*getValue(nextStates[s_p]) );
            }
            outcomes.push_back(sum_s_p);
                    
            //Free memory
            nextStates.clear();
            reward.clear();
            probability.clear();
        }
                            
        //NOW we have the best valued action and simply display it
        best_a = arg_max(outcomes);
        ostr << "[";
        maze->DisplayAction(actions[best_a], ostr);
        ostr << "]";
                
        //Free memory
        outcomes.clear();
        actions.clear();
        
        //New line when row ends
        if(s.col == cols-1) ostr << endl;
    }
    
}
