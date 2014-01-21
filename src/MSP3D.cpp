#include "MSP3D.h"
#include "YenTopKShortestPathsAlg.h"
#include <set>
#include <octomap/ColorOcTree.h>
#include <cmath>

#include <sstream>


namespace msp{
MSP3D::MSP3D(octomap::OcTree &tree, int max_depth):m_tree(tree),m_path_found(false),m_alpha(1.0),m_eps(tree.getResolution()/10.0),m_max_tree_depth(max_depth),m_lambda1(0.999),m_lambda2(0.001) {
	m_M=100*pow(8,max_depth);
	m_epsilon=pow(0.5,1+3*m_max_tree_depth);
}

bool MSP3D::init(octomap::point3d start,octomap::point3d end){
	if(m_tree.coordToKeyChecked(start, m_max_tree_depth ,m_start) && m_tree.coordToKeyChecked(end, m_max_tree_depth ,m_end)){
		//m_current_point=m_start;
		m_current_coord=m_tree.keyToCoord(m_start);
		m_start_coord=m_tree.keyToCoord(m_start);
		m_end_coord=m_tree.keyToCoord(m_end);
		m_current_path.push_back(m_start_coord);
		m_nb_step=0;
		return true;
	}
	return false;
}

bool MSP3D::step(){
	std::cout << "Calculate Graph" << std::endl;
	reducedGraph();
//	std::set<kshortestpaths::BaseVertex*>  res;
//	std::map<int,int> neighboor_counts;
//	for(int i=0;i<m_nodes.size();++i){
//		res.clear();
//		m_graph.get_adjacent_vertices(m_graph.get_vertex(i), res);
//		if(neighboor_counts.count(res.size())==0){
//			neighboor_counts[res.size()]=1;
//		}else{
//			neighboor_counts[res.size()]=neighboor_counts[res.size()]+1;
//		}
//
//		//check graph
////		std::cout << std::endl << "vertex " << i << " ID " << m_graph.get_vertex(i)->getID() <<std::endl;
////		for(std::set<kshortestpaths::BaseVertex*>::iterator it=res.begin(),end=res.end();it!=end;++it){
////			//kshortestpaths::BaseVertex* k=*it;
////			std::cout << "neighboor with " << (*it)->getID() <<std::endl;
////			res.clear();
////		}
//	}
//	std::cout << std::endl << "neighboor count check" << std::endl;
//	std::cout << "nb neighboors => nb_nodes" << std::endl;
//	for (std::map<int,int>::iterator it=neighboor_counts.begin(); it!=neighboor_counts.end(); ++it){
//	    std::cout << it->first << " => " << it->second << std::endl;
//	}
	// shortest path
	kshortestpaths::YenTopKShortestPathsAlg yenAlg(m_graph, m_graph.get_vertex(m_start_index),m_graph.get_vertex(m_end_index));
	bool got_next=false;
	std::cout << "Solve shortest path" << std::endl;
	if(m_visited.count(m_current_coord)==0){
		std::cout << "First visit" << std::endl;
		if(yenAlg.has_next()){
			std::cout << "Existing path" << std::endl;
			got_next=true;
			m_visited[m_current_coord]=0;
		}
	}else{
		for(int i=0;i<m_visited[m_current_coord];++i){
			if(yenAlg.has_next()){
				yenAlg.next();
			}
		}
		if(yenAlg.has_next()){
			got_next=true;
		}
	}
	std::cout << "End of step update" << std::endl;
	//if solution
	if(got_next){
		std::cout << "shortest path found" << std::endl;
		//go forward // if goal return false;
		kshortestpaths::BasePath* result =yenAlg.next();
		std::stringstream it_name;
		it_name << "iteration" << m_nb_step << ".ot";
		visu(std::string(it_name.str()),result);
		std::cout << "Cost: " << result->Weight() << " Length: " << result->length() << std::endl;
		if(result->Weight()>=m_M){
			//no path without obstacles from current to finish
			return false;
		}
		for(int i=0;i<result->length();++i)
		{
			std::cout << m_nodes[result->GetVertex(i)->getID()].first;
			std::cout << "->";
		}
		std::cout << std::endl <<  "*********************************************" << std::endl;
		int next_point_id=result->GetVertex(1)->getID();
		//do stuff to prepare next iteration
		m_visited[m_current_coord]=	m_visited[m_current_coord]+1;

//		std::cout<<"before adding element"<< std::endl;
//		for(std::deque<octomap::point3d>::iterator it=m_current_path.begin(),end=m_current_path.end();it!=end;++it){
//			std::cout<< (*it) << std::endl;
//		}

		m_current_path.push_back(m_nodes[next_point_id].first);

//		std::cout<<"after adding element"<< std::endl;
//		for(std::deque<octomap::point3d>::iterator it=m_current_path.begin(),end=m_current_path.end();it!=end;++it){
//			std::cout<< (*it) << std::endl;
//		}



		if(getPathCost()>=m_M){
			//no path without obstacles from start to current
			return false;
		}
		//m_current_point;
		m_current_coord=m_nodes[next_point_id].first;

		if(next_point_id==m_end_index){
			std::cout << "goal reached" << std::endl;
			m_path_found=true;
			return false;
		}else{
			return true;
		}
	}else{
		std::cout << "shortest path not found" << std::endl;
		//go back // if path empty => no solution return false
		m_visited[m_current_coord]=0;
		m_current_path.pop_back();
		if(m_current_path.size()==0){
			//no possible path
			return false;
		}else{
			m_current_coord=m_current_path.back();
			return true;
		}
	}
}

bool MSP3D::run(){
	while(step()){++m_nb_step;}
	if(m_path_found){
		return true;
	}else{
		return false;
	}
}

std::deque<octomap::point3d> MSP3D::getPath(){return m_current_path;}

double MSP3D::getPathCost(){
	double cost=0;
	std::deque<octomap::point3d>::iterator it=m_current_path.begin(),end=m_current_path.end();
	++it;
	for(0;it!=end;++it){
		cost += low_cost(*it);
		std::cout << low_cost(*it) << " -> ";
	}
	std::cout<<std::endl;
	return cost;
}

bool MSP3D::inPath(octomap::point3d pt){
//	std::cout << "in path intro" << std::endl;
//	for(std::deque<octomap::point3d>::iterator it=m_current_path.begin(),end=m_current_path.end();it!=end;++it){
//				std::cout<< (*it) << std::endl;
//			}


	//TODO : change it to if it includes an element of the path, i e, if is_in(*it, pt, pt_size)

	for(std::deque<octomap::point3d>::iterator it=m_current_path.begin(),end=m_current_path.end();it!=end;++it){

//		std::cout << "in path test " <<pt << (*it) << std::endl;
		if(pt==(*it) && !((*it)==m_current_path.back())){
//			std::cout << "in path " << std::endl;
			return true;
		}
	}
	return false;
}

void MSP3D::reducedGraph(){
	m_graph.clear();
	m_nodes.clear();
	m_start_index=-1;
	m_end_index=-1;
	octomap::OcTree::tree_iterator it_end=m_tree.end_tree();
	bool skip=false;
	int depth=0;
	for(octomap::OcTree::tree_iterator it=m_tree.begin_tree();it!=it_end;++it){
		if(skip){
			if(it.getDepth()<=depth){
				skip=false;
			}
		}
		if(!skip){
			if((it.getCoordinate()-m_current_coord).norm()>m_alpha*it.getSize()  || it.isLeaf()){
				if(!inPath(it.getCoordinate())){
					m_nodes.push_back(std::pair<octomap::point3d,double>(it.getCoordinate(),it.getSize()));
				}
				skip=true;
				depth=it.getDepth();
			}
		}
	}
	int l=m_nodes.size();

	std::cout<< "number of nodes: " << l << std::endl;
	for(int i=0;i<l;++i){
		// !!!!!!!!!!!!!  if not in path?
//		std::cout<< "node " << i << ":" << m_nodes[i].first <<std::endl;
		m_graph.add_vertex(i);
		if(is_start(m_nodes[i])){
//			std::cout<<"start: "<< m_nodes[i].first <<std::endl;
			if(m_start_index!=-1){
				std::cout << "2 start nodes, fail" << std::endl;
				exit(1);
			}
			m_start_index=i;
		}
		if(is_goal(m_nodes[i])){
//			std::cout<<"end: "<< m_nodes[i].first <<std::endl;
			if(m_end_index!=-1){
				std::cout << "2 end nodes, fail" << std::endl;
				exit(1);
			}
			m_end_index=i;
		}
	}
	if(m_start_index==-1){
		std::cout << "0 start node, fail" << std::endl;
	}
	if(m_end_index==-1){
		std::cout << "0 end node, fail" << std::endl;
	}
	for(int i=0;i<l;++i){
		for(int j=i+1;j<l;++j){
				if(neighboor(m_nodes[i],m_nodes[j])){
//					std::cout<< "neighboor:" << i << "," << j <<std::endl;
//					std::cout<< "cost:" << cost(i,j) <<std::endl;
					m_graph.add_edge(i,j,cost(i,j));
					m_graph.add_edge(j,i,cost(j,i));
				}
		}
	}
}

void MSP3D::setObstacles(std::vector<std::pair<octomap::point3d,double> > obstacles){
	m_obstacles=obstacles;
}

octomap::OcTreeNode* MSP3D::findNode(octomap::point3d pt){
	for(octomap::OcTree::tree_iterator it = m_tree.begin_tree(),	end=m_tree.end_tree(); it!= end; ++it)
		{
			if(it.getCoordinate()==pt){
				return &(*it);
			}
		}
}

void MSP3D::visu(std::string filename, kshortestpaths::BasePath* path){
	octomap::ColorOcTree tree(0.1);  // create empty tree with resolution 0.1

	int mm=32;
	for (int x=-mm; x<mm; x++) {
		for (int y=-mm; y<mm; y++) {
			for (int z=-mm; z<mm; z++) {
				octomap::point3d endpoint ((float) x*100.0f, (float) y*100.0f, (float) z*100.0f);
				tree.updateNode(endpoint, false); // integrate 'free' measurement
			}
		}
	}

	for(octomap::ColorOcTree::tree_iterator it = tree.begin_tree(),	end=tree.end_tree(); it!= end; ++it)
	{
		if(it.getDepth()>=m_max_tree_depth){
			for(int i=0;i<8;++i){
				if(it->childExists(i)){
					it->deleteChild(i);
				}
			}
			//octomap::point3d vec_dir(1,1,1);
//			it->setLogOdds(octomap::logodds(1));
			//it->setLogOdds(octomap::logodds((vec_dir.cross(it.getCoordinate())).norm()/max_size));
		}
	}
	tree.updateInnerOccupancy();

	octomap::ColorOcTreeNode* n;
//	std::cout << "starting to create color tree" << std::endl;
	for(octomap::ColorOcTree::tree_iterator it = tree.begin_tree(),	end=tree.end_tree(); it!= end; ++it)
	{
		for(int i=0;i<m_nodes.size();++i){
			if(it.getCoordinate()==m_nodes[i].first){
//				std::cout << "change occupancy of node" << findNode(it.getCoordinate())->getOccupancy() << std::endl;
				//it->setColor(0,0,(char)floor(125+125*(findNode(it.getCoordinate())->getOccupancy())));
				it->setColor(0,0,255);
				it->setLogOdds(findNode(it.getCoordinate())->getLogOdds());
//				std::cout << "delete childrem" << std::endl;
				for(int i=0;i<8;++i){
					if(it->childExists(i)){
						it->deleteChild(i);
					}
				}
//				std::cout << "delete children done" << std::endl;
			}
		}
	}

	tree.updateInnerOccupancy();
//	for(int i=0;i<m_nodes.size();++i){
////		n = tree.updateNode(m_nodes[i].first, true);
////		n->setColor(0,0,200);
//		std::cout << "change color/get node" << std::endl;
//		//n=tree.setNodeColor(m_nodes[i].first.x(),m_nodes[i].first.y(),m_nodes[i].first.z(),0,0,(char)floor(255*(1-n->getOccupancy())));
//		n=tree.setNodeColor(m_nodes[i].first.x(),m_nodes[i].first.y(),m_nodes[i].first.z(),0,0,255);
//		if(n==NULL){
//			std::cout<< "NULL" << std::endl;
//		}
//		std::cout << "change occupancy of node" << n->getOccupancy() << std::endl;
//		n->setValue(1);
//		n->setColor(0,0,255);
//		std::cout << "delete childrem" << std::endl;
//		for(int i=0;i<8;++i){
//			if(n->childExists(i)){
//				n->deleteChild(i);
//			}
//		}
//		std::cout << "delete children done" << std::endl;
//	}
//	std::cout << "end of create color tree" << std::endl;
	//tree.prune();

    std::ofstream binary_outfile( filename.c_str(), std::ios_base::binary);

    if (!binary_outfile.is_open()){
      std::cout<<"Filestream to "<< filename << " not open, nothing written."<<std::endl;
      exit(1);
    }

	tree.writeBinary(binary_outfile);


	//previous path
	binary_outfile /*<< std::endl << "ppath" */<< m_current_path.size();
//	std::cout << std::endl << "ppath" << m_current_path.size();
	for(std::deque<octomap::point3d>::iterator it=m_current_path.begin(),end=m_current_path.end();it!=end;++it){
		it->writeBinary(binary_outfile);
//		std::cout << *it <<std::endl;
	}

	//future path
	binary_outfile /*<< std::endl << "fpath"*/ << path->length();
//	std::cout << std::endl << "fpath" << m_current_path.size();
	for(int i=0;i<path->length();++i){
		m_nodes[path->GetVertex(i)->getID()].first.writeBinary(binary_outfile);
		binary_outfile << m_nodes[path->GetVertex(i)->getID()].second;
//		std::cout << m_nodes[path->GetVertex(i)->getID()].first <<std::endl;
	}

	//start
	//binary_outfile << std::endl << "start";
	m_start_coord.writeBinary(binary_outfile);
//	std::cout << "start " << m_start_coord <<std::endl;
	//end
	//binary_outfile << std::endl << "end";
	m_end_coord.writeBinary(binary_outfile);
//	std::cout << "end " << m_end_coord <<std::endl;

	//obstacles
	binary_outfile /*<< std::endl << "fpath"*/ << m_obstacles.size();
//	std::cout << std::endl << "fpath" << m_current_path.size();
	for(int i=0;i<m_obstacles.size();++i){
		m_obstacles[i].first.writeBinary(binary_outfile);
		binary_outfile << m_obstacles[i].second;
//		std::cout << m_nodes[path->GetVertex(i)->getID()].first <<std::endl;
	}


	binary_outfile.close();

	//exit(0);
}

bool MSP3D::is_start(std::pair<octomap::point3d,double> &node){
	return is_in(m_current_coord,node);
}

bool MSP3D::is_goal(std::pair<octomap::point3d,double> &node){
	return is_in(m_end_coord,node);
}

bool MSP3D::is_in(octomap::point3d pt,std::pair<octomap::point3d,double> &node){
	double l=0.5*node.second;
	if(fabs(pt.x()-node.first.x())<l && fabs(pt.y()-node.first.y())<l && fabs(pt.z()-node.first.z())<l){
		return true;
	}
	return false;
}

double MSP3D::cost(int i, int j){
	double F=findNode(m_nodes[j].first)->getOccupancy();
//	std::cout<< "F: " << F << std::endl;
	if (F <= 1-m_epsilon){
//		std::cout << "normal cost: " << (m_lambda1*F+m_lambda2)*(1-pow(8,m_nodes[j].second/(m_tree.getResolution()*pow(2,16-m_max_tree_depth))))/(1-8) << std::endl;
//		std::cout << "node size: " << m_nodes[j].second << std::endl;
//		std::cout << "tree resolution: " <<  m_tree.getResolution()*pow(2,16-m_max_tree_depth) << std::endl;
		//return (m_lambda1*F+m_lambda2)*(1-pow(8,m_nodes[j].second/(m_tree.getResolution()*pow(2,16-m_max_tree_depth))))/(1-8);
		//return (m_lambda1*F+m_lambda2)*pow(8,m_nodes[j].second/(m_tree.getResolution()*pow(2,16-m_max_tree_depth))-1);
		//return (m_lambda1*F+m_lambda2)*8*pow(m_nodes[j].second/(m_tree.getResolution()*pow(2,16-m_max_tree_depth))-1,3);
		//return (m_lambda1*F+m_lambda2)*pow(m_nodes[j].second/(m_tree.getResolution()*pow(2,16-m_max_tree_depth)),3);
		return (m_lambda1*F+m_lambda2)*pow(m_nodes[j].second/m_tree.getNodeSize(m_max_tree_depth),3);
	}else{
//		std::cout << "obstacle cost: " << m_M << std::endl;
		return m_M;
	}
}

double MSP3D::low_cost(octomap::point3d pt){
	double F=findNode(pt)->getOccupancy();
	//std::cout << pt << F << std::endl;
	if (F <= 1-m_epsilon){
		return m_lambda1*F+m_lambda2;
	}else{
		return m_M;
	}
}

bool MSP3D::neighboor(std::pair<octomap::point3d,double> &na,std::pair<octomap::point3d,double> &nb){
	double l=0.5*(na.second+nb.second);
	int c=0;
	if(fabs(na.first.x()-nb.first.x())<=(l+m_eps) && fabs(na.first.y()-nb.first.y())<=(l+m_eps) && fabs(na.first.z()-nb.first.z())<=(l+m_eps)){
		if(fabs(na.first.x()-nb.first.x())<(l-m_eps)){
			++c;
		}
		if(fabs(na.first.y()-nb.first.y())<(l-m_eps)){
			++c;
		}
		if(fabs(na.first.z()-nb.first.z())<(l-m_eps)){
			++c;
		}
		if(c==2){
			return true;
		}
	}
	return false;
}

}
