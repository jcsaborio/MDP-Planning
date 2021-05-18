#include "maze.h"

Maze::Maze(PARAMS& params){
    cols = params.cols;
    rows = params.cols;
    traps = params.traps;
    p_traps = params.p_traps;
    goalstate = params.goal;
    
    //Change to use variable random seed
    //srand (time(NULL));
    srand (0);
    InitMaze();
}

/*
 * Initialize maze using given parameters
 */
void Maze::InitMaze(){
    //Set up grid tiles
    grid = new char*[rows];
    for(int i=0; i < rows; i++){
        grid[i] = new char[cols];
        for(int j=0; j < cols; j++){
            grid[i][j] = tile;
        }
    }
    
    //Place goal
    grid[goalstate->row][goalstate->col] = goal;
    
    //Place traps randomly around the grid
    int traps_placed = 0;
    while(traps_placed < traps){
        int c = rand() % cols;
        int r = rand() % rows;
        if(grid[r][c] == tile){ //Place traps only in empty tiles
            grid[r][c] = trap;
            traps_placed++;
        }
     }
}

/*
 * Expand an MDP state-action transition and list all resulting states, rewards and probabilities
 * 
 * The vectors are all of equal length and for each of them, position i corresponds to successor state i.
 */
void Maze::expandMDP(const State& origin, int action, vector<State>& nextStatesV, vector<double>& rewardV, vector<float>& probabilityV) const{
    //Assume transition is deterministic
    double prob = 1.0;
    double reward;
    
    //If on top of trap
    if(grid[origin.row][origin.col] == trap){
        State s(origin.row,origin.col);
        prob = 1 - p_traps; //Update escape probability
        probabilityV.push_back(p_traps);
        nextStatesV.push_back(s);
        rewardV.push_back(rTrap);
    }
    
    //If not on trap, add the one transition, reward, prob.
    State s(origin.row,origin.col);
    switch(action){
        case UP:
            if(s.row - 1 >= 0){
                s.row--;
                reward = rStep;
            }
            else{                
                reward = rOut;
            }
            break;
        case DOWN:
            if(s.row + 1 < rows){
                s.row++;
                reward = rStep;
            }
            else{
                reward = rOut;
            }            
            break;
        case LEFT:
            if(s.col - 1 >= 0){
                s.col--;
                reward = rStep;
            }
            else{
                reward = rOut;
            }
            break;
        case RIGHT:
            if(s.col + 1 < cols){
                s.col++;
                reward = rStep;
            }
            else{
                reward = rOut;
            }
            break;
    }
        
    // Find out if state is terminal, and assign reward
    if(goalstate->equals(s)){
        reward = rGoal;        
    }
    
    //Add all values to the vector.  If not on trap, they are all vectors with one element
    nextStatesV.push_back(s);
    probabilityV.push_back(prob);
    rewardV.push_back(reward);
}

/* 
 * List all MDP states, which in this case corresponds to every row,col pair
 */
void Maze::listStates(vector<State>& states) const{
    for(int i=0; i < rows; i++){
        for(int j=0; j < cols; j++){
            states.push_back(State(i,j));
        }
    }
}

/* 
 * Return all actions available in state s.
 * 
 * In this problem, all states have the same actions.
 */
void Maze::getActions(State& s, vector<int>& actions) const{
    for(int i=0; i < nActions; i++)
        actions.push_back(i);
}

void Maze::getLegalActions(State& s, vector<int>& actions) const{
    if(s.row > 0) actions.push_back(UP);
    if(s.row < rows-1) actions.push_back(DOWN);
    if(s.col > 0) actions.push_back(LEFT);
    if(s.col < cols-1) actions.push_back(RIGHT);
}

/* 
 * MDP Simulator
 * 
 * Simulate the transition from state s and action a
 */
bool Maze::Step(State& s, int action, double& reward) const{
    //Assume transition is not terminal
    bool terminal = false;        
    
    //Find out if agent landed on a trap
    if(grid[s.row][s.col] == trap){        
        //Simulate trap.  If true, agent remains trapped and cannot execute action.
        if(Bernoulli(p_traps)){
            reward = rTrap;
            return false; //Non-terminal state
        }
    }
    
    //Execute action if it is valid, and assign rewards
    switch(action){
        case UP:
            if(s.row - 1 >= 0){
                s.row--;
                reward = rStep;
            }
            else{
                reward = rOut;
            }
            break;
        case DOWN:
            if(s.row + 1 < rows){
                s.row++;
                reward = rStep;
            }
            else{
                reward = rOut;
            }            
            break;
        case LEFT:
            if(s.col - 1 >= 0){
                s.col--;
                reward = rStep;
            }
            else{
                reward = rOut;
            }
            break;
        case RIGHT:
            if(s.col + 1 < cols){
                s.col++;
                reward = rStep;
            }
            else{
                reward = rOut;
            }
            break;
    }
    
    // Find out if state is terminal, and assign reward
    if(goalstate->equals(s)){
        reward = rGoal;
        terminal = true;
    }
    
    return terminal;
}

int Maze::SelectRandom(State& s) const{
    vector<int> actions;
    getLegalActions(s, actions);
    
    int action = rand() % actions.size(); //Uniformly random action
    actions.clear();
    return action;
}

/*
 * Simulate a Bernoulli trial with a given probability
 */
bool Maze::Bernoulli(double p) const{
    return rand() < p * RAND_MAX;
}

/*** Output functions ***/

/*
 * Display a complete state including the grid and the agent
 */
void Maze::DisplayState(const State& state, std::ostream& ostr) const{
    for(int i=0; i < rows; i++){
        for(int j=0; j < cols; j++){
            ostr << " ";
            if(state.equals(i, j))
                ostr << agent;
            else
                ostr << grid[i][j];
            ostr << " ";
        }
        ostr << std::endl;
    }
}

/*
 * Display a character for each numerical action
 */
void Maze::DisplayAction(int action, std::ostream& ostr) const{
    switch(action){
        case UP: ostr << "U";
            break;
        case DOWN: ostr << "D";
            break;
        case LEFT: ostr << "L";
            break;
        case RIGHT: ostr << "R";
            break;        
    }
}
