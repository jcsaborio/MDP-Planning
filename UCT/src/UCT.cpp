#include "UCT.h"
#include "Statistic.h"

using std::ofstream;

//// Start Class NODE ////
Node::Node(const State& s, vector<int> actions){
    this->s = new State(s);
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

vector< vector<Node*> *> * Node::getSuccessorsVector(){
    return &successors;
}

void Node::setSuccessors(vector< vector<Node*> *> successors){
    cout << "Setting " << successors.size() << " sucs" << endl;
    successors = successors;
}

void Node::freeSuccessor(int action, State& s){    
    assert(action >= 0 && action < successors.size());    

    for(Node* n_ : *(successors[action])){
        if(s.equals(n_->getState())){
            n_ = NULL; //free pointer
        }
    }
}

bool Node::expanded(){
    return successors.size() > 0;
}

//Get successor that matches state s
Node* Node::getSuccessor(int action, State& s){
    if(successors.size() == 0){
        cout << "Node not expanded!" << endl;
        return 0;
    }    
    
    assert(action >= 0 && action < successors.size());
    Node * next = 0;

    for(Node* n_ : *(successors[action])){        
        if(s.equals(n_->getState())){
            next = n_;
        }
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
    actionCount[a] += 1;
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
    if(actionCount[a]) value = reward[a] / actionCount[a];
    else value = reward[a];
    
    return value;
}
        
State& Node::getState(){
    return *s;
}

//// End Class NODE ////

//// Start Class UCT ////
UCT::UCT(UCT_PARAMS& searchParams, EXP_PARAMS& expParams, Maze * maze){    
    this->searchParams.discount = searchParams.discount;
    this->searchParams.depth = std::ceil(DiscountDepth / std::log(searchParams.discount)); //search depth in whole steps
    this->searchParams.exploration = searchParams.exploration;
    
    this->searchParams.startstate = searchParams.startstate;
    this->searchParams.goalstate = searchParams.goalstate;
    
    this->expParams.minSims = expParams.minSims;
    this->expParams.maxSims = expParams.maxSims;
    this->expParams.numSteps = expParams.numSteps;
    this->expParams.numRuns = expParams.numRuns;
    this->expParams.verbose = expParams.verbose;
    this->expParams.outputFile = expParams.outputFile;
    
    this->MDP = maze;    
}

UCT::~UCT(){
    
}

/*
 * UCB1 action selection rule
 * greedy = true uses no exploration bias, for example to select an action after planning
 */
int UCT::UCB(Node * n, bool greedy){
    vector<int> bestA;
    vector<int> actions;
    double bestQ = -Infinity;
    MDP->getLegalActions(n->getState(), actions); //Get all legal actions in s
       
    for(auto a : actions){
        double q = n->getValue(a);
       
        if(!greedy){
            int N_ = n->getCount();
            int n_ = n->getActionCount(a);
            if(n_ == 0)
                q += Infinity;
            else
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
    actions.clear();
    
    return action;
}

/*
 * Plan with UCT from node n, nsims times.
 */
int UCT::Search(Node * n, int nsims){
        
    State s(n->getState());
    for(int i=0; i < nsims; i++){        
        double r = Simulate(s, n, searchParams.depth);
        s.copy(n->getState());
    }
    
    return UCB(n, true);
}

/*
 * Standard MCTS simulation step, until depth = 0
 */
double UCT::Simulate(State& s, Node* n, int depth){
    if(!depth) return 0;
    
    double reward = 0.0;
    double delayedReward = 0.0;
    double totalReward = 0.0;
    bool terminal = false;
    int action;
    
    action = UCB(n);
    terminal = MDP->Step(s, action, reward); //Simulate step with given action

    if(!n->expanded()){            
        expandNode(n); //Newly visited nodes get all successors added at once
    }
    
    if(!terminal){        
        Node* next = n->getSuccessor(action, s); //Get node ptr to resulting state        
        
        //If node has not been visited
        if(next->getCount() == 0){            
            State * nextState = new State(next->getState());
            
            //Perform MCTS Rollout
            delayedReward = Rollout(*nextState, depth-1);
            
            next->increaseCount();
            delete nextState;
        }
        else{
            //Continue search if state is not terminal and has been visited
            delayedReward = Simulate(s, next, depth-1);
        }
    }
    
    totalReward = reward + searchParams.discount*delayedReward;
    
    //Update counts and values
    n->increaseCount();
    n->increaseActionCount(action);
    n->addReward(action, totalReward);
        
    return totalReward;
}

/*
 * Standard MCTS Rollout function
 */
double UCT::Rollout(State& s, int depth){
    double reward = 0.0;    
    double totalReward = 0.0;
    double discount = 1.0;
    bool terminal = false;
    int action;
    
    for(int i=depth; i > 0 && !terminal; i--){
        action = MDP->SelectRandom(s); //Select action using RO policy
        terminal = MDP->Step(s, action, reward); //Simulate step in MDP
        
        totalReward += reward * discount; //Compute discounted return
        discount *= searchParams.discount;
    }
    
    if(expParams.verbose >= 2)
        cout << "Rollout finished with R = " << totalReward << endl;
    
    return totalReward;
}

/* 
 * Create and add all successors of node n
 */
void UCT::expandNode(Node * n){
   
    //Receive vector from node and edit directly
    vector< vector<Node *> *> * successors = n->getSuccessorsVector();
    
    vector<int> actions;
    vector<int> actions_s;
    MDP->getActions(n->getState(), actions); //Get all actions in s
    
    for(auto a : actions){
        successors->push_back(new vector< Node *>); //Create vector for action a

        vector<State> nextStates;
        vector<double> r;
        vector<float> p;
        //Get all possible states derived from a
        MDP->expandMDP(n->getState(), a, nextStates, r, p);        
                
        for(State& s : nextStates){
            //Create successor node with its own actions
            MDP->getActions(s, actions_s);
            Node * m = new Node(s, actions_s);
            //Add successor to vector of action a (last = current)
            successors->back()->push_back(m);
            actions_s.clear();
        }
        
        p.clear();
        r.clear();
        nextStates.clear();
    }

}

//// End Class UCT ////

/// Execution functions ///

/*
 * Single run, up to numSteps steps
 */
void UCT::Run(){
    
    double undiscountedReturn = 0.0;
    double discountedReturn = 0.0;
    double discount = 1.0;
    bool terminal = false;    
        
    vector<int> actions;
    MDP->getActions(*(searchParams.startstate), actions);

    //Create tree root
    Root = new Node(*(searchParams.startstate), actions);
    actions.clear();
    expandNode(Root);
        
    Node * n = Root;
    State& s(n->getState());
    int t;
    
    for(t=0; t < expParams.numSteps && !terminal; t++){
        
        if(expParams.verbose >= 2)
            cout << "Searching from root = " << n->getState() << ", count = " << n->getCount() << endl;
                
        double reward;        
        int action = Search(n, expParams.sims);        
        
        terminal = MDP->Step(s, action, reward); //Simulate step with action               
        
        undiscountedReturn += reward;
        discountedReturn += reward*discount;
        discount *= searchParams.discount;
                
        n = n->getSuccessor(action, s); //Transition to new node                
        s.copy(n->getState());

        if(expParams.verbose >= 1){
            cout << "A: ";
            MDP->DisplayAction(action, cout);
            cout << endl;
            
            cout << "R: " << reward << endl;
            
            cout << "S': \n";
            MDP->DisplayState(s, cout);
            cout << endl;
        }
        
        //Get successor node
        /*Node * m = n->getSuccessor(action, s);
        n->freeSuccessor(action, s); //Eliminate successor from list
        cout << "Freed" << endl;
        delete n; //Delete old tree
        cout << "Deleted" << endl;
        n = m; //continue planning with new node
        if(!n->expanded()){            
            expandNode(n); //Newly visited nodes get all successors added at once
        }
        s.copy(n->getState());*/        
    }
    
    delete Root;
    
    results.discountedReturn.push_back(discountedReturn);
    results.undiscountedReturn.push_back(undiscountedReturn);
    
    if(t == expParams.numSteps) cout << "\tTerminated (reached step limit). ";
    if(terminal) cout << "\tGoal reached. ";
}

/*
 * Multiple runs, up to numRuns.
 * Also keep track of the duration.
 */
void UCT::MultiRun(){
    for(int r=0; r < expParams.numRuns; r++){        
        cout << "Starting run " << r+1 << " with " << expParams.sims << " simulations." << endl;
        
        auto start = std::chrono::high_resolution_clock::now();
        Run();
        auto stop = std::chrono::high_resolution_clock::now();
        
        cout << "Disc. R. = " << results.discountedReturn[r] << ", Undisc. R. = " << results.undiscountedReturn[r] << endl;
        
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        results.time.push_back(duration.count());
    }
}

/*
 * Schedule a multi run for each no. of simulations specified
 * Also, collect and generate statistics, and print to outpufFile
 */
void UCT::Experiment(){
    double discMean, discStdErr, undiscMean, undiscStdErr, meanTime;
    
    ofstream outputFile;
    outputFile.open(expParams.outputFile.c_str());
    if(!outputFile.is_open())
        std::cerr << "Error opening file \"" << expParams.outputFile << "\"" << endl;

    outputFile << "\t\tUndiscounted\tDiscounted" << endl;
    outputFile << "Sims\tRuns\tReturn\tError\tReturn\tError\tTime" << endl;
    
    for(int i=expParams.minSims; i <= expParams.maxSims; i++){
        expParams.sims = 1 << i; //2^i simulations
        
        cout << "Using " << expParams.sims << " simulations. "<<endl;
        
        MultiRun();
        
        discMean = STATISTIC::mean(results.discountedReturn);
        discStdErr = STATISTIC::stdError(results.discountedReturn);
        
        undiscMean = STATISTIC::mean(results.undiscountedReturn);
        undiscStdErr = STATISTIC::stdError(results.undiscountedReturn);
        
        meanTime = STATISTIC::mean(results.time) / 1000;
        
        cout << "Mean disc. return = " << discMean << " +- " << discStdErr << endl;    
        cout << "Mean undisc. return = " << undiscMean << " +- " << undiscStdErr << endl;
        
        outputFile  << expParams.sims << "\t"
                    << expParams.numRuns << "\t"
                    << std::setprecision(4) << undiscMean << "\t"
                    << std::setprecision(4) << undiscStdErr << "\t"
                    << std::setprecision(4) << discMean << "\t"
                    << std::setprecision(4) << discStdErr << "\t"
                    << std::setprecision(4) << meanTime << "\t"
                    << endl;
                    
        results.clear();
    }
        
    outputFile.close();
}
