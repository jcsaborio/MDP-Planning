/* 
 * Utility functions for statistics
 * 
 * Juan Carlos Saborio, DFKI Labor Niedersachsen (2021)
 */


#ifndef STAT_H
#define STAT_H

#include <vector>
#include <cmath>

using std::vector;

namespace STATISTIC{

    double mean(vector<double> values){
        double mean = 0.0;
        for(auto v : values){
            mean += v;
        }
        mean /= values.size();
        return mean;
    }
    
    double variance(vector<double> values){
        double mu = mean(values);
        double var = 0.0;
        for(auto v : values){
            var += std::pow(v - mu, 2);
        }
        var /= values.size();
        return var;
    }
    
    double stdError(vector<double> values){
        return sqrt(variance(values) / values.size());
    }
    
};

#endif
