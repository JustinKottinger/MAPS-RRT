#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include "../includes/ReadWorld.h"

// prints the elements of the vectors filled via readFile()
// this is really only useful when wanting to how readFile() works
// also useful for quickly debugging readFile()
void print(std::vector<double> const &vec)
{
	for (int i = 0; i < vec.size(); i++)
	{
		std::cout << vec.at(i) << ' ';
	}
	std::cout << "" << std::endl;
}

void MyPrint(std::vector<double> vec)
{
	for (int i = 0; i < vec.size(); i++)
	{
		std::cout << vec[i] << ' ';
	}
	std::cout << "" << std::endl;
}


void print(std::vector<std::string> const &vec)
{
	for (int i = 0; i < vec.size(); i++)
	{
		std::cout << vec.at(i) << ' ';
	}
	std::cout << "" << std::endl;
}

// takes in a "world" file that includes info for workspace
// this function reads in the information and fills the data into vectors
void readFile(const char* filename, std::vector<double> &b, std::vector<double> &g, 
	std::vector<double> &obs, std::vector<double> &strt, std::string &DynamicModel, int &dimension,
	int &numVehicles, int &numControls, std::vector<std::string> &data)
{
	bool bflg = false;
	bool gflg = false;
	bool oflg = false;
	bool sflg = false;
	
	
	std::ifstream fin (filename, std::ifstream::in);
	if (!fin)
	{
		std::cout << "Could NOT open the file" << std::endl;
		return;
	}
	
	while (!fin.eof())
	{
		std::string str;
		fin >> str;
		if (str == "DynModel")
		{
			std::string strM;
			fin >> strM;
			DynamicModel = strM.c_str();
		}
		else if (str == "Dimension")
		{
			std::string strDim;
			fin >> strDim;
			dimension = atof(strDim.c_str());
		}
		else if (str == "NumVehicles")
		{
			std::string strV;
			fin >> strV;
			numVehicles = atof(strV.c_str());
		}
		else if (str == "NumControls")
		{
			std::string strC;
			fin >> strC;
			numControls = atof(strC.c_str());
		}
		// note dimension, numVehicles, and Dimension are only used for the rest of
		// 	world file infomraiton extraction
		else if (str == "Data")
		{
			for (int i = 0; i<(numVehicles*(dimension + numControls) + 1); i++)
			{
				std::string strD;
				fin >> strD;
				size_t len = strD.length();
				data.push_back(strD.c_str());
			}
		}
		else if (str == "boundary")
		{
			for (int i = 0; i<2*dimension; i++)
			{
				std::string strB;
				fin >> strB;
				size_t len = strB.length();
				b.push_back(atof(strB.c_str()));
				// std::cout << b[i] << std::endl;
			}
			bflg = true;
		}
		else if(str == "goal")
		{
			for (size_t i = 0; i < (dimension * numVehicles); i++)
			{
				std::string strG;
				fin >> strG;
				size_t len = strG.length();
				g.push_back(atof(strG.c_str()));
			}
			gflg = true;
		}
		else if (str == "obstacle")
		{
			for (int i = 0; i<6; i++)
			{
				std::string strO;
				fin >> strO;
				size_t len = strO.length();
				obs.push_back(atof(strO.c_str()));
			}
			oflg = true;
		}
		else if (str == "start")
		{
			for (int i = 0; i < (dimension * numVehicles); i++)
			{
				std::string strS;
				fin >> strS;
				size_t len = strS.length();
				strt.push_back(atof(strS.c_str()));
			}
			sflg = true;
		}
	}
	fin.close();
	if (!bflg)
		std::cout<<"No boundary is give"<< std::endl;
	if (!gflg)
		std::cout<<"NO goal is given"<< std::endl;		
	if (!oflg)
		std::cout<<"No obstacle is given"<< std::endl;
	if (!sflg)
		std::cout<<"No start is given"<< std::endl;	
	return;
}