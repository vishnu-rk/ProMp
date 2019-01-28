#include "promp.hpp"
#include "csvReader.hpp"
#include "dataHandle.hpp"
#include <fstream>

int main(){
	/**
	* vector of strings containing file names of all .csv files (each .csv file contains data from a single demonstration)
	*/
	std::vector<std::string> fileList;
	std::string filePath = "../etc/";
	fileList.push_back(filePath + "example1.csv");
	fileList.push_back(filePath + "example2.csv");
	fileList.push_back(filePath + "example3.csv");
	fileList.push_back(filePath + "example4.csv");
	/**
	* data handle object which takes in index of trajectory in .csv file and also the file list
	*/
	DataHandle pelvisData(1,fileList);
	/// get required trajectory from all demonstrations into a single matrix
	Eigen::MatrixXd dataBase1 = pelvisData.getData();	
	/// initialize promp object with number of basis functions and std as arguments.
	ProMP baseMp(dataBase1,35,0.0286);
	/// generate trajectory of required number of points with the generateTrajectory function.
	Eigen::VectorXd vect = baseMp.generateTrajectory(300);



	/// Below code is for writing all the data into an output.csv file for visualization.
	std::ofstream myfile;
	myfile.open ("output.csv");
	int j;
	int k;
	int plotlen;
	if (vect.innerSize()>dataBase1.outerSize())
		plotlen =vect.innerSize();
	else
		plotlen =dataBase1.outerSize();
	for (int i=0;i<plotlen;++i){
		if (i<dataBase1.outerSize())
			j=i;
		else 
			j=dataBase1.outerSize()-1;

		if (i<vect.innerSize())
			k=i;
		else 
			k=vect.innerSize()-1;
		myfile << dataBase1(0,j)<<","<<dataBase1(1,j)<<","<< dataBase1(2,j)<<","<< dataBase1(3,j)<<","<<vect(k)<<"\n";
	}
	myfile.close();
	return 0;
}