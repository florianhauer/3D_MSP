
#include "MSP3D.h"
#include <octomap_utils.h>

int main(int argc, char** argv) {
	int max_depth=3;
	double max_size=0.1*pow(2,16)*sqrt(3);
	octomap::OcTree tree(0.1);  // create empty tree with resolution 0.1
	int mm=32;
	for (int x=-mm; x<mm; x++) {
		for (int y=-mm; y<mm; y++) {
			for (int z=-mm; z<mm; z++) {
				octomap::point3d endpoint ((float) x*100.0f, (float) y*100.0f, (float) z*100.0f);
				tree.updateNode(endpoint, false); // integrate 'free' measurement
			}
		}
	}
	int c=0;
//	for(octomap::OcTree::tree_iterator it = tree.begin_tree(),	end=tree.end_tree(); it!= end; ++it)
//	{
//		if(it.getDepth()==2){
//			++c;
//		}
//	}
//	std::cout << "number of nodes of level 2: " << c <<std::endl;
	for(octomap::OcTree::tree_iterator it = tree.begin_tree(),	end=tree.end_tree(); it!= end; ++it)
	{
		if(it.getDepth()>=max_depth){
			for(int i=0;i<8;++i){
				if(it->childExists(i)){
					it->deleteChild(i);
				}
			}
			octomap::point3d vec_dir(1,1,1);
			//it->setLogOdds(octomap::logodds(1));
			it->setLogOdds(octomap::logodds((vec_dir.cross(it.getCoordinate())).norm()/max_size));
		}
	}
	tree.updateInnerOccupancy();
//	for(octomap::OcTree::tree_iterator it = tree.begin_tree(),	end=tree.end_tree(); it!= end; ++it)
//	{
////		std::cout<< it.getDepth()<<std::endl;
//		std::cout<< it.getCoordinate()<<std::endl;
////		std::cout<< it.getSize()<<std::endl;
////		std::cout<< it.isLeaf()<<std::endl;
//		std::cout << it->getOccupancy() << std::endl <<std::endl;
//	}
//	std::cout<< "Tree depth: " << tree.getTreeDepth() << std::endl;

	std::cout << "Create algo" << std::endl;
	msp::MSP3D algo(tree,max_depth);
	octomap::point3d start(-3000.0,-3000.0,-3000.0);
	octomap::point3d end(3000.0,3000.0,3000.0);
	std::cout << "Init algo" << std::endl;
	algo.init(start,end);
	std::cout << "Run algo" << std::endl;
	double scale=tree.getResolution()*pow(2,16-max_depth)/(1-8);
	if(algo.run()){
		std::deque<octomap::point3d> sol=algo.getPath();
		std::cout << "Path length: " << sol.size() << std::endl;
		std:;cout << "Path cost: " << algo.getPathCost() << std::endl;
		std::cout << "Path :" << std::endl;
		for(std::deque<octomap::point3d>::iterator it=sol.begin(),end=sol.end();it!=end;++it){
			std::cout << (((*it)*(1/scale))+octomap::point3d(3.5,3.5,3.5))*(1.0/7.0) << std::endl;
		}
	}else{
		std::cout << "No path exists between the given start and end points" << std::endl;
	}
	std::cout << "fin" << std::endl;

}
