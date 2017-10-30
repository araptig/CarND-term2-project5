#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi()
{
	return(M_PI);
}
double deg2rad(double x)
{
	return(x * pi() / 180);
}
double rad2deg(double x)
{
	return(x * 180 / pi());
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{//hasData
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos)
  {
    return "";
  } else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}//hasData

double polyeval(Eigen::VectorXd coeffs, double x)
{//evaluate polynomial
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++)
  {
    result += coeffs[i] * pow(x, i);
  }
  return(result);
}//evaluate polynomial

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,int order)
{//fit a polynomial
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++)
  {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++)
  {//for
    for (int i = 0; i < order; i++)
    {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }//for

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}//fit a polynomial

int main()
{// main
  const int poly_order 	= 3;
  const int latency    	= 0.1;
  const int latency_ms  = 1000*latency;
  //double Lf 		= 2.67;
  const bool with_latency_comp = true;

  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
    {
      string s = hasData(sdata);
      if (s != "")
      {//data
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry")
        {//telemetry
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];  //path in map coordinates
          vector<double> ptsy = j[1]["ptsy"];  //path in map coordinates
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];  // miles per hour
          v       *= 0.44704;        // meters per second

          vector<double> traj_car_x(ptsx.size());
          vector<double> traj_car_y(ptsy.size());
          for(unsigned int i = 0; i < ptsx.size(); i++)
          {//map coordinates --> car coordinates
              double shift_x = ptsx[i] - px;
              double shift_y = ptsy[i] - py;
              double cos_p   = cos(psi);
              double sin_p   = sin(psi);
              traj_car_x[i] =  shift_x * cos_p  +  shift_y * sin_p;
              traj_car_y[i] = -shift_x * sin_p  +  shift_y * cos_p;
           }

          Eigen::VectorXd coeffs;
          {//coefficients of trajectory in car coordinates
        	  // vector --> eigen
        	  double* ptrx = &traj_car_x[0];
        	  Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);
        	  double* ptry = &traj_car_y[0];
        	  Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);
        	  coeffs = polyfit(ptsx_transform, ptsy_transform, poly_order);
          }

          Eigen::VectorXd state(6);
          {// in car coordinates px=0, py=0, psi=0
              double cte  = coeffs[0];     		// f(0) = c_0
              double epsi = - atan(coeffs[1]);	// because px = 0

              if (with_latency_comp == false)
              {
            	    state << 0, 0, 0, v, cte, epsi;
              }
              else
              {//latency compensation
            	  //change of sign because turning left is negative sign in simulator but positive yaw for MPC (not doing it)

            	  // control signals
            	  double delta = j[1]["steering_angle"];    // minus because different convention
            	  delta *= -deg2rad(25);					// back to radians
            	  double a     = j[1]["throttle"];


            	  //delay model state by latency
            	  double v_l  = v*latency;
            	  double Lf = 2.67;
            	  double temp = v_l*delta/Lf;
            	  state[0] = v_l*cos(delta);   			//px
            	  state[1] = v_l*sin(delta);   			//py
            	  state[2] = delta + temp;				//psi
            	  state[4] = cte  + v_l*sin(epsi);     	//cte
            	  state[5] = epsi + temp;				//epsi
            	  state[3]  = v   + a*latency;          //vel
              }//latency compensation
          }//

          // get control signals, [0] is in degrees
          vector<double> control = mpc.Solve(state,coeffs);

          json msgJson;
          {//provide control signals and trajectories
        	  //(a) apply steering angle & throttle, normalized to be between [-1, 1]
        	  double steer_value    = -control[0]/ deg2rad(25);
        	  double throttle_value = control[1];
        	  msgJson["steering_angle"] = steer_value;
        	  msgJson["throttle"] = throttle_value;

        	  //(b) Display the MPC predicted trajectory
              msgJson["mpc_x"] = mpc.traj_x;
              msgJson["mpc_y"] = mpc.traj_y;

              //(c) Display reference trajectory
              //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
              // the points in the simulator are connected by a Yellow line
              msgJson["next_x"] = traj_car_x;
              msgJson["next_y"] = traj_car_y;
          }

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // latency because car does not actuate the commands instantly.
          this_thread::sleep_for(chrono::milliseconds(latency_ms));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }//data
      }//telemetry
      else
      {// Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t)
  {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    } else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length)
  {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}// main
