
#include "MSP3D.h"

int main(int argc, char** argv) {
	octomap::OcTree tree (0.1);  // create empty tree with resolution 0.1
	msp::MSP3D algo(tree);
	octomap::point3d start(0.0,0.0,0.0);
	octomap::point3d end(0.0,0.0,0.0);
	algo.init(start,end);
	if(algo.run()){
		algo.getPath();
	}else{
		std::cout << "No path exists between the given start and end points" << std::endl;
	}
	std::cout << "fin" << std::endl;

}
