#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	if( rmse(0) <= .09 &&
      	rmse(1) <= .10 &&
      	rmse(2) <= .40 &&
      	rmse(3) <= .30 ) {
		std::cout << "reached Benchmark index !" << std::endl;

    	std::cout << ":  rmse = " 
         << rmse(0) << "  " << rmse(1) << "  " 
         << rmse(2) << "  " << rmse(3) << endl
         << " all values are below tolerances of "
         << ".09, .10, .40, .30" << endl;



		  }


	//return the result
	return rmse; 

}