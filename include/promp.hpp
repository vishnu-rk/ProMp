/** 
*Author: Vishnu Radhakrishnan
*/

#ifndef PROMP
#define PROMP

#include <iostream>
#include <cmath>
#include <vector>
#include <iterator>
#include <string>
#include "phase.hpp"

using namespace Eigen;
/** Class for ProMP for single trajectory
*/
class ProMP{
public:
	/** \brief constructor
	*	\param data 	(numDemos_,trajLength_) data matrix
	*	\param numBf 	 number of basis functions
	*	\param stdBf	 standard deviation 
	*/
	ProMP(Eigen::MatrixXd& data,int numBf, double stdBf);
	/** \brief	generate trajectory
	*	\return trajectory
	*/
	Eigen::VectorXd generateTrajectory(int requiredTrajLen);
	/** \brief	generates average trajectory
	*	\return trajectory
	*/
	Eigen::VectorXd generateAvgTrajectory();
	/** \brief	set desired goal/end point for trajectory
	*	\param 	goal 	desired value at end
	*	\param	std 	desired standard deviation
	*	\return 
	*/
	void setGoal(double goal,double std);
	/** \brief	set desired start/initial point for trajectory
	*	\param 	start 	desired value at start
	*	\param	std   	desired standard deviation
	*	\return
	*/
	void setStart(double start,double std);
	/** \brief	set via point for trajectory
	*	\param 	t 			time at which via point is to be added (between 0 and 1.0)
	*	\param	viaPoint 	desired value at via point
	*	\param	std   		desired standard deviation
	*	\return 
	*/
	void addViaPoint(double t, double viaPoint, double std);
	/** \brief	get average starting value of demo trajectories
	*	\return average starting value
	*/
	double getAvgStart();
	/** \brief	get average ending value of demo trajectories
	*	\return average ending value
	*/
	double getAvgEnd();
private:
	/** number of basis functions
	*/
	int numBf_;
	/** standard deviation of basis functions
	*/
	double stdBf_;
	/** number of demonstrations
	*/
	int numDemos_;
	/** number of points in the entire trajectory
	*/
	int trajLength_;
	/** average start value of all demo trajectories
	*/
	double start_;
	/** average end value of all demo trajectories
	*/
	double end_;
	/** average trajectory directly from demonstrations
	*/
	Eigen::VectorXd means;
	/** (numDemos_,trajLength_) Matrix for storing demonstration trajectories.
	*/
	/** std::vector of Eigen::Vector3d for storing information for adding via points in the trajectory
	*  Eigen::Vector3d(0) - time at which the waypoint is to be added. 
	*						values 0 and 1.0 are time values for start and end respectively.
	*  Eigen::Vector3d(1) - actual value of the point to be added.
	*  Eigen::Vector3d(1) - standard deviation with which thw point is to be added in the trajectory
	*/
	std::vector<Eigen::Vector3d>viaPoints_;
	/// phase vector
	Eigen::VectorXd phase_;
	double dt_;
	Eigen::MatrixXd demoTrajs_;
	Eigen::VectorXd numBfMid_;
	Eigen::MatrixXd phi_;
	Eigen::MatrixXd w_;	
	Eigen::MatrixXd phaseDiff_;
	Eigen::VectorXd meanW_;
	Eigen::MatrixXd stdW_;
	/** \brief	train using demonstration trajectories to obtain average weights
	*	\param 	dataBase 	matrix of demonstration trajectories
	*	\return 			basis functions
	*/
	void train(Eigen::MatrixXd& dataBase);
	/** \brief	computes phase
	*	\param phaseSpeed < 1 slower than average trajectory
	*			phaseSpeed > 1 faster than average trajectory
	*/
	void computePhase(double phaseSpeed);
	/** \brief	linear interpolator
	*	\param start - start value
	*	\param end - end value
	*	\param number - number of divisions
	*	\return vector of equidistant values
	*/
	Eigen::VectorXd linspace(double start, double end, int number);
	/// utility function
	Eigen::MatrixXd func(Eigen::MatrixXd& phaseDiff);
	/** \brief	generates basis functions
	*	\param phase
	*	\param length of required trajectory
	*	\return matrix of basis functions
	*/
	Eigen::MatrixXd generateBasisFunction(Eigen::VectorXd& phase, int trajLength);
};

#endif