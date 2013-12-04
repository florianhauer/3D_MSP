#include "MSP3D.h"


namespace msp{
MSP3D::MSP3D(octomap::OcTree &tree):m_tree(tree),m_path_found(false),m_alpha(1) {}

bool MSP3D::init(octomap::point3d start,octomap::point3d end){
	if(m_tree.coordToKeyChecked(start,m_start) && m_tree.coordToKeyChecked(end,m_end)){
		m_current_point=m_start;
		return true;
	}
	return false;
}

bool MSP3D::step(){
	reducedGraph();
	// shortest path
	//if solution
	//go forward // if goal return true
	return false;
	//else
	//go back // if path empty => no solution return false
}

bool MSP3D::run(){
	while(step()){}
	if(m_path_found){
		return true;
	}else{
		return false;
	}
}

std::deque<octomap::OcTreeKey> MSP3D::getPath(){return m_current_path;}

void MSP3D::reducedGraph(){
	m_graph=kshortestpaths::Graph();
	m_nodes.clear();
	octomap::OcTree::tree_iterator it_end=m_tree.end_tree();
	bool skip=false;
	int depth=0;
	for(octomap::OcTree::tree_iterator it=m_tree.begin_tree();it!=it_end;++it){
		if(skip){
			if(it.getDepth()==depth){
				skip=false;
			}
		}
		if(!skip){
			if((it.getCoordinate()-m_current_coord).norm()>m_alpha*it.getSize()){
				m_nodes.push_back(std::pair<octomap::OcTreeKey,double>(it.getKey(),it.getSize()));
				skip=true;
				depth=it.getDepth();
			}
		}
	}
	int l=m_nodes.size();
	for(int i=0;i<l;++i){
		//m_graph.addVertex(i);
		for(int j=i+1;j<l;++j){
				if(neighboor(m_nodes[i],m_nodes[j])){
					//m_graph.addEdge(i,j);
					//m_graph.addEdge(j,i);
				}
		}
	}
}

bool MSP3D::neighboor(std::pair<octomap::OcTreeKey,double> &na,std::pair<octomap::OcTreeKey,double> &nb){
	return true;
}

void MSP3D::kthShortestPath(){}
}
