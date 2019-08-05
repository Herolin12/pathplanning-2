#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "behaviour.h"
#include "vehicle.h"
#include "helpers.h"
#include "mapping.h"
#include "constants.h"
#include "spline.h"

// for convenience
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;
using std::vector;
using std::cout;
using std::sort;

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

Behaviour::Behaviour(Vehicle ego, double ref_vel){
    this->s = ego.s;
    this->d = ego.d;
    this->lane = ego.lane;
    this->current_state = ego.state;
    this->ref_vel = ref_vel;
}

vector<string> Behaviour::available_states(){
    vector<string> states;
    if (this->current_state == "KL"){
        states = {"KL", "PLCL", "PLCR"};
    }
    else if (this->current_state == "PLCL"){
        if (this->lane > 0){
            states = {"KL", "PLCL","LCL"};
        }
        else if (this->lane == 0){
            states = {"KL", "PLCR"};
        }
    }
    else if (this->current_state == "PLCR"){
        if (this->lane < 2){
            states = {"KL", "PLCR", "LCR"};
        }
        else if (this->lane == 2){
            states = {"KL", "PLCL"};
        }
    }
    else if ((this->current_state == "LCL") || (this->current_state == "LCR")){
        states = {"KL"};
    }

    return states;
}

/*
vector<Trajectory> Behaviour::generate_trajectory(vector<Vehicle> predictions){

}

 */



