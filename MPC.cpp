#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Global parameters used both be GF_eval & MPC
size_t N = 10;							// steps
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval
{//FG_eval
 public:
  Eigen::VectorXd coeffs;					//polynomial coefficients

  const double ref_v            = 40;		//velocity reference
  const double dt               = 0.1;		//sample interval
  const double Lf	 			= 2.67;     //car size

  // cost weights
  const double wt_cte 			= 1;
  const double wt_orientation   = 5;
  const double wt_velocity      = .1;
  const double wt_steer_angle	= 0;
  const double wt_steer_angle_d = 0.3;
  const double wt_acc           = 0;
  const double wt_acc_d         = 0;

  FG_eval(Eigen::VectorXd _coeffs)
  {//construct
	  coeffs       		= _coeffs;
  }

  // define ADvector to interact with CppAD
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  // `fg`  contains the cost [0] and constraints.
  // `vars` contains the variable values (state & actuators)
  // operator determines fg from vars
  void operator()(ADvector& fg, const ADvector& vars)
  {//operator

	  // (I) cost = fg[0]
	  fg[0] = 0;
	  for (unsigned int t = 0; t < N; t++)
	  {//cost due to cte/orientation/velocity
		  fg[0] += wt_cte*CppAD::pow(vars[cte_start + t], 2);		 		// cte
	      fg[0] += wt_orientation*CppAD::pow(vars[epsi_start + t], 2);		// orientation
	      fg[0] += wt_velocity*CppAD::pow(vars[v_start + t] - ref_v, 2); 	// velocity mismatch penalty
	  }

	  // Minimize the use of actuators.
	  for (unsigned int t = 0; t < N - 1; t++)
	  {//cost due to control signal level
		  fg[0] += wt_steer_angle*CppAD::pow(vars[delta_start + t], 2);  	// delta amplitude penalty
	      fg[0] += wt_acc*CppAD::pow(vars[a_start + t], 2);			// acceleration amplitude penalty
	  }

	  // cost due to large differential actuations (i.e. abrupt changes)
	  for (unsigned int t = 0; t < N - 2; t++)
	  {
		  fg[0] += wt_steer_angle_d * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
	      fg[0] += wt_acc_d * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
	  }

	    // (II) Constraints
	    // (a) initial constraints:
	    fg[1 + x_start] 	= vars[x_start];
	    fg[1 + y_start] 	= vars[y_start];
	    fg[1 + psi_start] 	= vars[psi_start];
	    fg[1 + v_start] 	= vars[v_start];
	    fg[1 + cte_start] 	= vars[cte_start];
	    fg[1 + epsi_start] 	= vars[epsi_start];

	    // (b) rest of constraints:
	    for (unsigned int t = 1; t < N; t++)
	    {// trajectory points
	    	// The idea here is to constraint this value to be 0

	    	// The state at time t+1 .
	    	AD<double> x1 = vars[x_start + t];
	    	AD<double> y1 = vars[y_start + t];
	    	AD<double> psi1 = vars[psi_start + t];
	    	AD<double> v1 = vars[v_start + t];
	    	AD<double> cte1 = vars[cte_start + t];
	    	AD<double> epsi1 = vars[epsi_start + t];

	    	// The state at time t.
	    	AD<double> x0 = vars[x_start + t - 1];
	    	AD<double> y0 = vars[y_start + t - 1];
	    	AD<double> psi0 = vars[psi_start + t - 1];
	    	AD<double> v0 = vars[v_start + t - 1];
	    	AD<double> cte0 = vars[cte_start + t - 1];
	    	AD<double> epsi0 = vars[epsi_start + t - 1];

	    	// Only consider the actuation at time t.
	    	AD<double> delta0;
	    	AD<double> a0;
	    	if(t==1)
	    	{
	    		delta0 = vars[delta_start + t - 1];
	    		a0 = vars[a_start + t - 1];
	    	}
	    	else
	    	{// use previous actuations (to account for latency)
	    		a0 = vars[a_start + t - 2];
	    	    delta0 = vars[delta_start + t - 2];
	    	}

	    	AD<double> f0 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*x0*x0 + coeffs[3]*x0*x0*x0;
	    	AD<double> psides0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*x0*x0);

	    	// equations for the model:
	    	// x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
	    	// y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
	    	// psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
	    	// v_[t+1] = v[t] + a[t] * dt
	    	// cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
	    	// epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
	    	fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
	    	cout << "fg " << fg[1 + x_start + t] << " " <<  - (v0 * CppAD::cos(psi0) * dt) << endl;
	    	fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
	    	fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);//  + -> - for steering angle
	    	fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
	    	fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
	    	fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
	    }// trajectory points
  	}//operator
  };//FG_eval


MPC::MPC()
{
	max_steering_angle_rad = (25.0/180.0) * M_PI;
	max_acc                = 1.0;
	max_value              = 1.0e19;

	// N time-steps  N-1 actuations
	n_vars 		  = 6*N + (N-1)*2;
	n_constraints = 6*N;
}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{//solve
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // for easier notation
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v =  state[3];
  double cte = state[4];
  double epsi = state[5];

  // (a) vars: independent variables
  Dvector vars(n_vars);
  {//vars
	  for (unsigned int i = 0; i < n_vars; i++)
	  {
		  vars[i] = 0;
	  }

	  // Set the initial variable values
	  vars[x_start] 	= x;
	  vars[y_start] 	= y;
	  vars[psi_start] 	= psi;
	  vars[v_start] 	= v;
	  vars[cte_start] 	= cte;
	  vars[epsi_start] 	= epsi;
  }//vars

  // (b) bounds on vars
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  {//vars bounds
    	for (unsigned int i = 0; i < delta_start; i++)
    	{// non-actuators bounds set to the max /pm values
    		vars_lowerbound[i] = -max_value;
    		vars_upperbound[i] =  max_value;
    	}

    	for (unsigned int i = delta_start; i < a_start; i++)
    	{// bounds of delta in radians
    		vars_lowerbound[i] = -max_steering_angle_rad;
    		vars_upperbound[i] =  max_steering_angle_rad;
    	}

    	// Acceleration/decceleration bounds
    	for (unsigned int i = a_start; i < n_vars; i++)
    	{
    		vars_lowerbound[i] = -max_acc;
    		vars_upperbound[i] =  max_acc;
    	}
  }//vars bounds

  // (c) bounds on constraints
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  {//constraint bounds
    for (unsigned int i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;
  }// constraint bounds

  // (d) options for IPOPT solver (did not change)
  std::string options;
  {//options
	  // Uncomment this if you'd like more print information
	  options += "Integer print_level  0\n";
	  // NOTE: Setting sparse to true allows the solver to take advantage
	  // of sparse routines, this makes the computation MUCH FASTER. If you
	  // can uncomment 1 of these and see if it makes a difference or not but
	  // if you uncomment both the computation time should go up in orders of
	  // magnitude.
	  options += "Sparse  true        forward\n";
	  options += "Sparse  true        reverse\n";
	  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
	  // Change this as you see fit.
	  options += "Numeric max_cpu_time          0.5\n";
  }//optiotns

  // (e) define return vector called solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // construct fg_eval class
  FG_eval fg_eval(coeffs);

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // (a) check status
  bool ok = (solution.status == CppAD::ipopt::solve_result<Dvector>::success);
  if (ok == false)
  {
	  cout << "ok = " << ok << endl;
  }

  // (b) print cost
  auto cost = solution.obj_value;
  cout << "Cost " << cost << endl << endl;

  // (c) store predicted trajectory
  traj_x = {};
  traj_y = {};
  for (unsigned int i = 0; i < N; i++)
  {
      traj_x.push_back(solution.x[x_start + i]);
      traj_y.push_back(solution.x[y_start + i]);
  }

  //Return the first actuator values. The variables can be accessed with
  return {solution.x[delta_start],   solution.x[a_start]};
}//solve
