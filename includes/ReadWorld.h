#include <fstream>
#include <string>
#include <iostream>
#include <vector>

// txt/Shaull/2agents/Linear/example3_2Linear.txt

void readFile(std::string filename, std::vector<double> &b, std::vector<double> &g, 
	std::vector<double> &obs, std::vector<double> &strt, std::string &DynamicModel, int &dimension,
	int &numVehicles, int &numControls, std::vector<std::string> &data);

void print(std::vector<double> const &vec);

void MyPrint(std::vector<double> vec);


void print(std::vector<std::string> const &vec);

int getSolutionCost();