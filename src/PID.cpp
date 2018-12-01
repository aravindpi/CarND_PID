#include "PID.h"
#include <limits>
#include <iostream>

using namespace std;

/*
 * Complete the PID class.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  // Initialize Gains
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  // Initialize errors
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  // state of the filter
  prev_cte = 0.0;
  
  // twiddle gain management
  dp = {0.1*Kp, 0.1*Kd, 0.1*Ki};
  paramIdx = 2;
  dpIncreased = false;
  dpDecreased = false;

  // twiddle management & errors
  step = 1;
  run_error = 0.0;
  best_error = std::numeric_limits<double>::max(); 
  
}

void PID::UpdateError(double cte) {

  // update errors
  p_error = cte;
  d_error = cte - prev_cte;
  i_error += cte;

  // update filter state
  prev_cte = cte;

  // twiddle
  {
    return;
    if (step % 4000 > 100) {
      // update error after an intitial settling period
      run_error += cte*cte;
    }

    // Check error against previous best error every N steps
    if (step % 4000 == 0) {
      
      if (run_error < best_error) {
	// If error is smaller than previous best
	//   - update best
	//   - twiddle the next gain
	best_error = run_error;
	if (step > 4000){ dp[paramIdx] *= 1.1;}
	paramIdx = (paramIdx + 1) % 3;
	dpDecreased = false;
	dpIncreased = false;
      }
      if (!dpDecreased && !dpIncreased) {
	// Twiddle - Increase Gain
	switch (paramIdx) {
	case 0:
	  Kp += dp[paramIdx];
	  break;
	case 2:
	  Ki += dp[paramIdx];
	  break;
	case 1:
	  Kd += dp[paramIdx];
	  break; 
	}
	dpIncreased = true;
      } else if (dpIncreased && !dpDecreased){
	// Twiddle - Decrease Gain
	switch (paramIdx) {
	case 0:
	  Kp -= 2*dp[paramIdx];
	  break;
	case 2:
	  Ki -= 2*dp[paramIdx];
	  break;
	case 1:
	  Kd -= 2*dp[paramIdx];
	  break;
	}
	dpDecreased = true;
      } else {
	// Twiddle - Re-Increase Gain
	switch (paramIdx) {
	case 0:
	  Kp += dp[paramIdx];
	  break;
	case 2:
	  Ki += dp[paramIdx];
	  break;
	case 1:
	  Kd += dp[paramIdx];
	  break; 
	}
	dp[paramIdx] *= 0.9;
	paramIdx = (paramIdx + 1) % 3;
	dpIncreased = false;
	dpDecreased = false;
      }
      std::cout << "--------------------------------" << std::endl;
      std::cout << "Iteration# " <<  step/4000 << std::endl;
      std::cout << "Run Error: " <<  run_error << std::endl;
      std::cout << "Best Error: " <<  best_error << std::endl;
      std::cout << "Kp: " << Kp << std::endl;
      std::cout << "Ki: " << Ki << std::endl;
      std::cout << "Kd: " << Kd << std::endl;
      std::cout << "--------------------------------" << std::endl;
      run_error = 0;
    }
    
    step += 1;
  }
}

double PID::TotalError() {
  return p_error*Kp + i_error*Ki + d_error*Kd;
}

