#include "promp.hpp"

ProMP::ProMP(Eigen::MatrixXd& data,int numBf, double stdBf):demoTrajs_(data),numBf_(numBf),stdBf_(stdBf){
	numDemos_ = demoTrajs_.innerSize();
	trajLength_ = demoTrajs_.outerSize();
	dt_ = 1.0/trajLength_;
	numBfMid_ = linspace(0,1,numBf_);
	computePhase(1.0);
	phi_ = generateBasisFunction(phase_,trajLength_);
	train(data);
	}

Eigen::VectorXd ProMP::linspace(double start, double end, int number){
	Eigen::VectorXd result(number);
	double factor = (end - start)/(number -1);
	for (int i=0;i<number;++i){
		result(i)=i*factor;
		}
	return result;
	}

void ProMP::computePhase(double phaseSpeed){
	int timeSteps = (int)(trajLength_/phaseSpeed);
	Phase phase(dt_,phaseSpeed,timeSteps);
	phase_ = phase.getPhase();
	}

Eigen::MatrixXd ProMP::generateBasisFunction(Eigen::VectorXd& phase, int trajLength){
	Eigen::MatrixXd phaseDiff0 = phase.replicate(1,numBf_).transpose();	
	Eigen::MatrixXd numBfMidMat = numBfMid_.replicate(1,trajLength);
	Eigen::MatrixXd phaseDiff2 = phaseDiff0 - numBfMidMat;
	Eigen::MatrixXd phaseDiff3 = func(phaseDiff2).transpose();
	Eigen::VectorXd sumBf = phaseDiff3.rowwise().sum();
	Eigen::MatrixXd phaseDiff6(trajLength,numBf_);
	for (int i=0; i< trajLength ;++i){
		for (int j=0; j< numBf_; ++j){
			phaseDiff6(i,j)= phaseDiff3(i,j)/sumBf(i);
			}
		}
	auto phi = phaseDiff6.transpose();
	return phi;
	}

void ProMP::train(Eigen::MatrixXd& dataBase){
	double rand = 0.000000000001;
	Eigen::MatrixXd W_(numDemos_,numBf_);
	for (int i=0; i<dataBase.innerSize();++i){
		auto wDemoTraj = (((phi_*phi_.transpose()).inverse()+rand* MatrixXd::Identity(numBf_,numBf_))*(phi_*dataBase.row(i).transpose())).transpose();
		W_.row(i) = wDemoTraj;
		}
	w_= W_;
	Eigen::VectorXd meanW = W_.colwise().mean();
	meanW_ = meanW;
	Eigen::MatrixXd covMat = w_;
	MatrixXd centered = covMat.rowwise() - covMat.colwise().mean();
	MatrixXd cov = (centered.adjoint() * centered) / double(covMat.rows() - 1);
	stdW_ = cov; 
	means = dataBase.colwise().mean();
	start_ = means(0);
	end_   = means(trajLength_-1);
	}

void ProMP::addViaPoint(double t, double viaPoint, double std){
	Eigen::Vector3d viaPoint_;
	viaPoint_(0) = t;
	viaPoint_(1) = viaPoint;
	viaPoint_(2) = std;
	viaPoints_.push_back(viaPoint_);
	}

void ProMP::setStart(double start,double std){
	addViaPoint(0.0,start,std);
	}

void ProMP::setGoal(double goal,double std){
	addViaPoint(1.0,goal,std);	
	}

Eigen::VectorXd ProMP::generateTrajectory(int requiredTrajLen){
	double phaseSpeed = 1.0;
	auto newMeanW = meanW_;
	auto newStdW = stdW_;
	int timeSteps = (int)(trajLength_/phaseSpeed);
	Phase newPhase(dt_,phaseSpeed,timeSteps);
	auto newPhase_ = newPhase.getPhase();
	auto phi = generateBasisFunction(newPhase_,trajLength_);
	for(int i=0;i<viaPoints_.size();++i){
		auto phase1 = newPhase.getPhaseFromTime(viaPoints_[i][0]);
		auto phiT = generateBasisFunction(phase1,1);
		Eigen::VectorXd sig(1);
		sig(0) = viaPoints_[i][2];
		Eigen::VectorXd point(1);
		point(0) = viaPoints_[i][1];
		auto aux = sig + (phiT.transpose()*newStdW)*phiT;
		newMeanW = newMeanW + (newStdW*phiT/aux(0))*(point - phiT.transpose()*newMeanW);
		newStdW = newStdW - (newStdW*phiT/aux(0)) * (phiT.transpose()*newStdW);
		}
	auto sampleW = newMeanW;
	auto gentrajDemoLen =  phi.transpose()*sampleW;
	double delta = (trajLength_-1)*1.0/(requiredTrajLen);
	Eigen::VectorXd gentrajReqLen(requiredTrajLen);
	for (int i=0;i<requiredTrajLen;++i){
		gentrajReqLen(i) = gentrajDemoLen((int)(i*delta));
		}
	return gentrajReqLen;
	}

Eigen::VectorXd ProMP::generateAvgTrajectory(){
	setStart(means(0),0.000001);
	setGoal(means(trajLength_-1),0.000001);
	return generateTrajectory(trajLength_);
	}


Eigen::MatrixXd ProMP::func(Eigen::MatrixXd& phaseDiff){
	Eigen::MatrixXd phaseDiffF(phaseDiff.innerSize(),phaseDiff.outerSize());
	for(int i=0;i<phaseDiff.innerSize();++i){
		for(int j=0;j<phaseDiff.outerSize();++j){
			auto x = phaseDiff(i,j);
			phaseDiffF(i,j) = std::exp(-0.5*pow(x/stdBf_,2))/(sqrt(2*3.14)*stdBf_);
			}
		}
	return phaseDiffF;
	}

double ProMP::getAvgStart(){
	return start_;
	}

double ProMP::getAvgEnd(){
	return end_;
	}









