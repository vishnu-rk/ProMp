/** 
*Author: Vishnu Radhakrishnan
*/
#ifndef DATA_HANDLE
#define DATA_HANDLE
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <string>
#include "csvReader.hpp"
/** Class for handling data from multiple demonstrations.
* Data(all trajectories) from each demonstration should be stored in an individual .csv file 
* Each column within .csv file represents a trajectory.
* All .csv files must have trajectories(columns) in the same sequence.
*/
class DataHandle{
private:
	/// approximate length
	int reqTrajLen_;
	/// column number of desired trajectory in each csv file.
	int index_;
	/// file names with trajectories for each demonstration.
	std::vector<std::string> fileList_;
	/// std::vector of Eigen::VectorXd of trajectories
	std::vector<Eigen::VectorXd> trajList_;
	/// Eigen::Matrix of trajectories
	Eigen::MatrixXd data_;
public:
	/** \brief				 handle data from multiple demonstrations
	*	\param	reqTrajLen   required length of trajectory
	*	\param	index    	 column of required trajectory in each csv file
	*	\param	fileList  	 vector of .csv file names	
	*/
	DataHandle(int index,std::vector<std::string> fileList):index_(index),fileList_(fileList){
		std::vector<int> trjLens;
		for(int i=0;i<fileList_.size();++i){
			CSVReader reader(fileList_[i]);
			Eigen::VectorXd data = reader.get1Ddata(index_,reader.getData());
			trjLens.push_back(data.innerSize());
			trajList_.push_back(data);
			}
		int minLen = *std::min_element(trjLens.begin(), trjLens.end());
		reqTrajLen_=minLen;
		double delta;
		Eigen::MatrixXd data1(fileList_.size(),reqTrajLen_);
		for(int i=0;i<trajList_.size();++i){			
			delta = (trajList_[i].innerSize() - 1)*1.0/(reqTrajLen_-1);
			for(int j=0;j<reqTrajLen_;++j){
				data1(i,j)=trajList_[i]((int)(j*delta));
				}
			}
		data_= data1;
		}
	/** \brief	get required data
	*/
	Eigen::MatrixXd getData(){
		return data_;
		}
};
#endif