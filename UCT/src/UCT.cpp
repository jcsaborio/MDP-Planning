#include "UCT.h"

/// Class NODE
Node::Node(State * s, vector<int> actions){
    this->s = new State(*s);
    this->actions = actions;
    actionCount.resize(actions.size(), 0);
    reward.resize(actions.size(), 0);
    count = 0;    
}

Node::~Node(){
    for(vector<Node*>* s_ : successors){
        for(Node* n_ : *(s_)){
            delete n_;
        }
    }
    reward.clear();
    successors.clear();
    actionCount.clear();
    actions.clear();
}

void Node::setSuccessors(vector< vector<Node*> *> successors){
    successors = successors;
}

//Get successor that matches state s
Node* Node::getSuccessor(int action, State& s){
    assert(action >= 0 && action < actions.size());
    Node * next = 0;
    for(Node* n_ : *(successors[action])){
        if(s.equals(n_->getState()))
            next = n_;
    }
    return next;
}
        
int Node::getAction(int a){
    assert(a >= 0 && a < actions.size());
    return actions[a];
}

int Node::getCount(){
    return count;
}

void Node::increaseCount(){
    count++;
}
        
void Node::increaseActionCount(int a){
    assert(a >= 0 && a < actions.size());
    actionCount[a]++;
}

int Node::getActionCount(int a){
    assert(a >= 0 && a < actions.size());
    return actionCount[a];
}

void Node::addReward(int a, double r){
    assert(a >= 0 && a < actions.size());
    reward[a] += r;
}

double Node::getValue(int a){
    assert(a >= 0 && a < actions.size());
    double value = 0.0;    
    if(count) value = reward[a] / actionCount[a];
    else value = reward[a];
    
    return value;
}
        
State& Node::getState(){
    return *s;
}

//Node* getSuccessor(int n);
///End Class NODE

UCT::UCT(UCT_PARAMS& searchParams, EXP_PARAMS& expParams, Maze * maze){    
    searchParams.discount = searchParams.discount;
    searchParams.depth = std::ceil(DiscountDepth / std::log(searchParams.discount)); //search depth in whole steps
    searchParams.startstate = searchParams.startstate;
    searchParams.goalstate = searchParams.goalstate;
    searchParams.exploration = searchParams.exploration;
    
    expParams.minSims = expParams.minSims;
    expParams.maxSims = expParams.maxSims;
    expParams.numSteps = expParams.numSteps;
    expParams.numRuns = expParams.numRuns;
    
    MDP = maze;
    
    //Create root
    //vector<int> actions;
    //MDP->getActions(*(searchParams.startstate), actions);
    //Root = new Node(searchParams.startstate, actions);
    //actions.clear();
    
    //Add root succesors (in sims)
    //expandNode(Root);
    
    cout << "UCT created" << endl;
}

UCT::~UCT(){
    delete Root;
    delete MDP;
}

int UCT::UCB(Node * n, bool greedy){
    vector<int> bestA;
    vector<int> actions;
    double bestQ = -Infinity;
    MDP->getActions(n->getState(), actions); //Get all actions in s
    
    vector<double> values;
    for(auto a : actions){
        double q = n->getValue(a);
        
        if(!greedy){
            int N_ = n->getCount();
            int n_ = n->getActionCount(a);
            q += searchParams.exploration * std::sqrt(std::log(N_ + 1) / n_);
        }
        
        if (q >= bestQ){
            if (q > bestQ) bestA.clear();
            bestQ = q;
            bestA.push_back(a);
        }

    }
    
    int best = rand() % bestA.size();
    int action = bestA[best];
    bestA.clear();
    
    return action;
}

int UCT::Search(Node * n, int nsims){
    State s(n->getState());
    for(int i=0; i < nsims; i++)
        Simulate(s, n, searchParams.depth);
    return UCB(n, true);
}

double UCT::Simulate(State& s, Node* n, int depth){
    if(!depth) return 0;
    
//    State s;
//    s.copy(n->getState());    
    double reward = 0.0;
    double delayedReward = 0.0;
    double totalReward = 0.0;
    bool terminal = false;
    int action;
    
    //If not yet visited, continue with rollout
    if(n->getCount() == 0){
        n->increaseCount();
        expandNode(n); //Newly visited nodes get all successors added at once
        reward = Rollout(n->getState(), depth-1);
    }
    else{
        action = UCB(n);
        terminal = MDP->Step(s, action, reward); //Simulate step with action
        
        Node* next = n->getSuccessor(action, s); //Get node of resulting state
        
        if(!terminal)
            delayedReward = Simulate(s, next, depth-1); //Continue search if state is not terminal
    }
    
    totalReward = reward + searchParams.discount*delayedReward;
    
    n->increaseActionCount(action);
    n->addReward(action, totalReward);
    
    return totalReward;
}

double UCT::Rollout(State& s, int depth){
    double reward = 0.0;    
    double totalReward = 0.0;
    double discount = 1.0;
    bool terminal = false;
    
    for(int i=depth; i > 0 && !terminal; i--){
        int action = MDP->SelectRandom(s);
        terminal = MDP->Step(s, action, reward);
        
        totalReward += reward * discount;
        discount *= searchParams.discount;
    }
    
    return totalReward;
}

// Create and add all successors of node n
void UCT::expandNode(Node * n){
    vector< vector<Node *> *> successors;
    
    vector<int> actions;
    vector<int> actions_s;
    MDP->getActions(n->getState(), actions); //Get all actions in s

    for(auto a : actions){
        successors.push_back(new vector< Node *>); //add vector for action a
        
        vector<State> nextStates;
        vector<double> r;
        vector<float> p;
        MDP->expandMDP(n->getState(), a, nextStates, r, p);        
        
        for(State& s : nextStates){
            MDP->getActions(s, actions_s);
            Node * n = new Node(s, actions_s);
            successors.back()->push_back(n); //add successor to vector for action a (last = current)
            actions_s.clear();
        }
        
        p.clear();
        r.clear();
        nextStates.clear();
    }
    
    n->setSuccessors(successors); //Add to node n
}

//// 

void UCT::Run(){
    
    double undiscountedReturn = 0.0;
    double discountedReturn = 0.0;
    double discount = 1.0;
    bool terminal = false;
    
    
    cout << "Creating start state" << endl;
    vector<int> actions;
    MDP->getActions(*(searchParams.startstate), actions);
       
    Root = new Node(searchParams.startstate, actions);
    actions.clear();
    
    cout << "Node created " << Root->getState().row << ", " << Root->getState().col << endl;
    
    Node * n = Root;
    State& s(n->getState());
    
    
    for(int t=0; t < expParams.numSteps && !terminal; t++){
        double reward;                
        
        int action = Search(n, expParams.sims);        
        
        terminal = MDP->Step(s, action, reward); //Simulate step with action
        
        undiscountedReturn += reward;
        discountedReturn += reward*discount;
        discount *= searchParams.discount;
        
        MDP->DisplayAction(action, cout);
        MDP->DisplayState(s, cout);
        
        n = n->getSuccessor(action, s); //Transition to new node
        s.copy(n->getState());        
    }
}

void UCT::MultiRun(){
    for(int r=0; r <= expParams.numRuns; r++){
        cout << "Starting run " << r+1 << endl;
        Run();
    }
}

void UCT::Experiment(){
    results.clear();
    
    for(int i=expParams.minSims; i <= expParams.maxSims; i++){
        expParams.sims = 1 << i; //2^i simulations
        
        cout << "Using " << expParams.sims << " simulations. "<<endl;
        
        MultiRun();
    }
}
