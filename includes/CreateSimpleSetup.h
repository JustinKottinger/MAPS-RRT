#include <string>

namespace ob = ompl::base;
namespace oc = ompl::control;

void CreateSimpleSetup(oc::SimpleSetupPtr& ss, std::vector<double> bndry, 
	std::vector<double> gol, std::vector<double> strt, 
	std::string model, const double toll);