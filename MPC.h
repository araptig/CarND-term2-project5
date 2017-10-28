#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC
{
public:
	MPC();
	virtual ~MPC();

	double max_steering_angle_rad;
	double max_acc;
	double max_value;
	size_t n_vars;
	size_t n_constraints;

	vector<double> traj_x;	//trajectory x component
	vector<double> traj_y;	//trajectory y component

	// Return the first actuatotions.
	vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif
