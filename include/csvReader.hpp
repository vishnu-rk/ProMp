/** 
*Author: Vishnu Radhakrishnan
*/
#ifndef CSV_READER
#define CSV_READER
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <fstream>
/** Class for handling data from .csv files
*/
class CSVReader{
private:
	std::string fileName_;
	std::string delimeter_; 
	std::vector<std::vector<std::string> > dataList_;
	std::vector<std::vector<double>> list;
public:
	CSVReader(std::string filename, std::string delm = ","):fileName_(filename), delimeter_(delm){ }
	Eigen::MatrixXd getData(){
	std::ifstream file(fileName_.c_str()); 
	std::string line = "";
	while (getline(file, line)){
		std::vector<std::string> vec;
		boost::algorithm::split(vec, line, boost::is_any_of(delimeter_));
		dataList_.push_back(vec);
		}
	file.close();	
	for(std::vector<std::string> vec : dataList_){
		std::vector<double> oneLine;
		for(std::string data : vec){
			oneLine.push_back(std::stod(data));
			}
		list.push_back(oneLine);		
		}
	Eigen::MatrixXd data(list.size(),list[0].size());
	for (int row = 0; row < list.size(); ++row){
   		for (int col = 0; col < list[0].size(); ++col){
        	data(row,col) = list[row][col];
   			}
		}
	return data;
	}
	Eigen::VectorXd get1Ddata(int index,Eigen::MatrixXd data_){
	Eigen::VectorXd data(list.size());
	for (int row = 0; row < data.innerSize(); ++row){   		
        	data(row) = data_(row,index) ;		
		}
	return data;
	}

};

#endif