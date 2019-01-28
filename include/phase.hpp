/** 
*Author: Vishnu Radhakrishnan
*/
#ifndef PHASE_H
#define PHASE_H
#include <Eigen/Dense>
#include <Eigen/Core>
/** Class for adjusting trajectory speed
*/
class Phase{
public:
	/** \brief				constructor
	*	\param dt 			= 0.005
	*	\param phaseSpeed 	<1.0 for slow and  >1.0 for fast
	*	\param timeSteps 	= required length of trajectory 
	*/
	Phase(double dt, double phaseSpeed, int timeSteps):dt_(dt),phaseSpeed_(phaseSpeed),timeSteps_(timeSteps){
		double phaseStart_ = -dt_;
		phaseEnd_ = phaseStart_ + timeSteps_*dt_;
		Eigen::VectorXd dz = Eigen::RowVectorXd::Ones(timeSteps_)*phaseSpeed_;
		Eigen::VectorXd z = cumSum(dz)*dt;
		z_ = z;
	}
	/** \brief 				get phase for a specific point in the trajectory
	*	\param				time step of which phase is to be found
	*	\return 			phase
	*/
	Eigen::VectorXd getPhaseFromTime(int timeSteps){
		Eigen::VectorXd phaseN(1);
		phaseN(0) = timeSteps/phaseEnd_;
		return phaseN;
	}
	/** \brief 				get phase 
	*	\return 			phase
	*/
	Eigen::VectorXd getPhase(){
		return z_;
	}
	/// phase vector
	Eigen::VectorXd z_;	
private:
	double dt_;	
	int timeSteps_;	
	double phaseSpeed_;	
	double phaseEnd_;	
	Eigen::VectorXd cumSum(Eigen::VectorXd vect){
		Eigen::VectorXd sumArr(vect.innerSize());
		double acc = 0;
		for(int i = 0; i < vect.innerSize(); i++){
	 		acc += vect(i);
	 		sumArr(i) = acc;
		}
		return sumArr;
	}
};

#endif