
#include "MSP3D.h"
#include <octomap_utils.h>
#include <AbstractOcTree.h>
//#include <ctime>

void depth_traversal(octomap::OcTreeNode* node,octomap::point3d coord, double size){
	std::vector<octomap::point3d> m_child_dir;
	m_child_dir.push_back(octomap::point3d(-1,-1,-1));
	m_child_dir.push_back(octomap::point3d(1,-1,-1));
	m_child_dir.push_back(octomap::point3d(-1,1,-1));
	m_child_dir.push_back(octomap::point3d(1,1,-1));
	m_child_dir.push_back(octomap::point3d(-1,-1,1));
	m_child_dir.push_back(octomap::point3d(1,-1,1));
	m_child_dir.push_back(octomap::point3d(-1,1,1));
	m_child_dir.push_back(octomap::point3d(1,1,1));
	std::cout << coord << " , " << size << " , " << node->getOccupancy() << " , " << node->hasChildren() <<std::endl;
	if(size>1000){
		for(int i=0;i<8;++i){
			depth_traversal(node->getChild(i),coord+m_child_dir[i]*0.25*size,size*0.5);
		}
	}
}

int main(int argc, char** argv) {

	int max_depth=16;
	std::vector<std::pair<octomap::point3d,double> > obstacles;
	obstacles.push_back(std::pair<octomap::point3d,double>(octomap::point3d(0,0,0),0.0001));

	std::cout<< "Loading tree" << std::endl;

	//load tree from file
	//octomap::OcTree* tree = new octomap::OcTree("/home/florian/workspace/3D_MSP/build/maps/octomap_CAMVID-textured.ot");
	octomap::AbstractOcTree* tree1 = octomap::AbstractOcTree::read("/home/florian/workspace/3D_MSP/build/maps/octomap_CAMVID-textured.ot");

	octomap::OcTree* tree=(octomap::OcTree*)tree1;

	std::cout<< "Tree loaded" << std::endl;

	tree->updateInnerOccupancy();

	//depth_traversal(tree->getRoot(),octomap::point3d(0,0,0),tree->getNodeSize(0));

	for(octomap::OcTree::tree_iterator it = tree->begin_tree(),	end=tree->end_tree(); it!= end; ++it)
	{
		if(it.getDepth()<max_depth){
			if(it->hasChildren()){
				for(int i=0;i<8;++i){
					if(!it->childExists(i)){
						it->createChild(i);
					}
				}
			}
		}
		if(it.getDepth()>=max_depth){
			for(int i=0;i<8;++i){
				if(it->childExists(i)){
					it->deleteChild(i);
				}
			}
		}
	}

//	std::ofstream binary_outfile( "copytree.bt", std::ios_base::binary);
//	tree->writeBinary(binary_outfile);

//	for(octomap::OcTree::tree_iterator it = tree->begin_tree(),	end=tree->end_tree(); it!= end; ++it)
//	{
//		for(int j=0;j<=it.getDepth();++j){
//			std::cout << "\t";
//		}
////		std::cout<< it.getDepth()<<std::endl;
//		std::cout<< it.getCoordinate()<<std::endl;
////		std::cout<< it.getSize()<<std::endl;
////		std::cout<< it.isLeaf()<<std::endl;
//
//		for(int j=0;j<=it.getDepth();++j){
//			std::cout << "\t";
//		}
//		std::cout << it->getOccupancy() <<std::endl;
//	}

	std::cout << "Create algo" << std::endl;
	msp::MSP3D algo(*tree,max_depth);
	octomap::point3d start(-3.0,70.0,0.0);
	octomap::point3d end(3.0,-70,0.0);
	std::cout << "Init algo" << std::endl;
	algo.setObstacles(obstacles);
	algo.setVisu(false);
	algo.init(start,end);
	std::cout << "Run algo" << std::endl;
	double scale=tree->getResolution()*pow(2,16-max_depth)/(1-8);
	clock_t tstart = clock();
	if(algo.run()){
		std::cout << "Time to run: " << (clock()-tstart)*1.0/CLOCKS_PER_SEC << std::endl;
		std::deque<octomap::point3d> sol=algo.getPath();
		std::cout << "Path length: " << sol.size() << std::endl;
		std::cout << "Path cost: " << algo.getPathCost() << std::endl;
//		std::cout << "Path :" << std::endl;
//		for(std::deque<octomap::point3d>::iterator it=sol.begin(),end=sol.end();it!=end;++it){
//			std::cout << (((*it)*(1/scale))+octomap::point3d(3.5,3.5,3.5))*(1.0/7.0) << std::endl;
//			std::cout << *it << std::endl;
//		}
	}else{
		std::cout << "Time to run: " << (clock()-tstart)*1.0/CLOCKS_PER_SEC << std::endl;
		std::cout << "No path exists between the given start and end points" << std::endl;
	}
	std::cout << "fin" << std::endl;
	delete tree;
}
