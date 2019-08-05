#ifndef JMT_H
#define JMT_H

#include <vector>
#include "Eigen-3.3/Eigen/Dense"


using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

vector<double> JMT(vector<double> start, vector<double> end, double T){
    double a_0 = start[0];
    double a_1 = start[1];
    double a_2 = start[2]/2;
    
    MatrixXd A = MatrixXd(3, 3);
    A << pow(T, 3), pow(T, 4), pow(T, 5),
        3*pow(T, 2), 4*pow(T, 3), 5*pow(T, 4),
        6*T, 12*pow(T,2), 20*pow(T, 3);

    MatrixXd B = MatrixXd(3,1);
    B << end[0]-(start[0]+start[1]*T+.5*start[2]*pow(T, 2)),
        end[1]-(start[1]+start[2]*T),
        end[2]-start[2];
    
    MatrixXd delta = A.inverse()*B;
    
    vector<double> results;
    results = {start[0], start[1], .5*start[2]};

    for(int i = 0; i < delta.size(); ++i) {
        results.push_back(delta.data()[i]);
    }
    
    return results;
}

vector<vector<double>> get_jmt(vector<double> initial, vector<double> final, double T){
    double si = initial[0];
    double si_d = initial[1];
    double si_dd = initial[2];
    double di = initial[3];
    double di_d = initial[4];
    double di_dd = initial[5];
    double sf = final[0];
    double sf_d = final[1];
    double sf_dd = final[2];
    double df = final[3];
    double df_d = final[4];
    double df_dd = final[5];

    vector<double> start_s = {si, si_d, si_dd};
    vector<double> end_s = {sf, sf_d, sf_dd};
    vector<double> start_d = {di, di_d, di_dd};
    vector<double> end_d = {df, df_d, df_dd};

    vector<double> jmt_s = JMT(start_s, end_s, T);
    vector<double> jmt_d = JMT(start_d, end_d, T);

    vector<vector<double>> output = {jmt_s, jmt_d};
    
    return output;
}

#endif