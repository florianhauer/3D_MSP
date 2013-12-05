
#include "MSP3D.h"

int main(int argc, char** argv) {
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
	for(octomap::OcTree::tree_iterator it = tree.begin_tree(),	end=tree.end_tree(); it!= end; ++it)
	{
		if(it.getDepth()==2){
			++c;
		}
	}
	std::cout << "number of nodes of level 2: " << c <<std::endl;
	for(octomap::OcTree::tree_iterator it = tree.begin_tree(),	end=tree.end_tree(); it!= end; ++it)
	{
		if(it.getDepth()>=3){
			for(int i=0;i<8;++i){
				if(it->childExists(i)){
					it->deleteChild(i);
				}
			}
		}
	}
	for(octomap::OcTree::tree_iterator it = tree.begin_tree(),	end=tree.end_tree(); it!= end; ++it)
	{
		std::cout<< it.getDepth()<<std::endl;
		std::cout<< it.getCoordinate()<<std::endl;
		std::cout<< it.getSize()<<std::endl;
		std::cout<< it.isLeaf()<<std::endl;
	}
	std::cout<< "Tree depth: " << tree.getTreeDepth() << std::endl;

	std::cout << "Create algo" << std::endl;
	msp::MSP3D algo(tree);
	octomap::point3d start(-3000.0,-3000.0,-3000.0);
	octomap::point3d end(3000.0,3000.0,3000.0);
	std::cout << "Init algo" << std::endl;
	algo.init(start,end);
	std::cout << "Run algo" << std::endl;
	if(algo.run()){
		std::deque<octomap::point3d> sol=algo.getPath();
		std::cout << "Path length: " << sol.size() << std::endl;
		std::cout << "Path :" << std::endl;
		for(std::deque<octomap::point3d>::iterator it=sol.begin(),end=sol.end();it!=end;++it){
			std::cout << *it << std::endl;
		}
	}else{
		std::cout << "No path exists between the given start and end points" << std::endl;
	}
	std::cout << "fin" << std::endl;

}
