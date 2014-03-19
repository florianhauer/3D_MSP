#include "MSP3D.h"
#include "YenTopKShortestPathsAlg.h"
#include "DijkstraShortestPathAlg.h"
#include <set>
#include <octomap/ColorOcTree.h>
#include <cmath>
#include <numeric>
#include <sstream>
#include <stdexcept>


namespace msp{
MSP3D::MSP3D(octomap::OcTree &tree, int max_depth):m_tree(tree),m_path_found(false),m_alpha(1.0),m_eps(tree.getResolution()/10.0),m_max_tree_depth(max_depth),m_lambda1(0.999),m_lambda2(0.001) {
	m_M=100*pow(8,max_depth);
	m_epsilon=pow(0.5,1+3*m_max_tree_depth);
	m_child_dir.push_back(octomap::point3d(-1,-1,-1));
	m_child_dir.push_back(octomap::point3d(1,-1,-1));
	m_child_dir.push_back(octomap::point3d(-1,1,-1));
	m_child_dir.push_back(octomap::point3d(1,1,-1));
	m_child_dir.push_back(octomap::point3d(-1,-1,1));
	m_child_dir.push_back(octomap::point3d(1,-1,1));
	m_child_dir.push_back(octomap::point3d(-1,1,1));
	m_child_dir.push_back(octomap::point3d(1,1,1));
}

bool MSP3D::init(octomap::point3d start,octomap::point3d end){
	if(m_tree.coordToKeyChecked(start, m_max_tree_depth ,m_start) && m_tree.coordToKeyChecked(end, m_max_tree_depth ,m_end)){
		//m_current_point=m_start;
		m_current_coord=m_tree.keyToCoord(m_start);
		m_start_coord=m_tree.keyToCoord(m_start);
		m_end_coord=m_tree.keyToCoord(m_end);
		m_current_path.clear();
		m_path_cost.clear();
		m_current_path.push_back(m_start_coord);
		m_misleading.clear();
		m_misleading[m_current_coord]=std::set<octomap::point3d,Point3D_Less>();
		m_nb_step=0;
		m_nb_backtrack=0;
//		std::stringstream it_name;
//		it_name << "iteration" << m_nb_step << ".ot";
//		visu_init(std::string(it_name.str()));
//		std::cout << it_name.str() << std::endl;
		m_nb_step++;
		return true;
	}else{
		std::cout << "start or goal not on map" << std::endl;
		exit(1);
	}
	return false;
}

bool MSP3D::step(){
//	std::cout << "Calculate Graph" << std::endl;
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
//	std::cout << "Solve shortest path" << std::endl;
/*	if(m_visited.count(m_current_coord)==0){
//		std::cout << "First visit" << std::endl;
		if(yenAlg.has_next()){
//			std::cout << "Existing path" << std::endl;
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
	}*/
	if(yenAlg.has_next()){
		got_next=true;
	}
	//return false;
//	std::cout << "End of step update" << std::endl;

	//TODO: check if the path contains obstacles (cost > M) if yes change get next to false

	//if solution
	if(got_next){
		//go forward // if goal return false;
		kshortestpaths::BasePath* result =yenAlg.next();
//		std::cout << "Cost: " << result->Weight() << " Length: " << result->length() << std::endl;
//		std::stringstream it_name;
//		it_name << "iteration" << m_nb_step << ".ot";
//		visu(std::string(it_name.str()),result);
//		std::cout << it_name.str() << std::endl;
		if(result->Weight()>=m_M){
			//no path without obstacles from current to finish
//			std::cout << "shortest path with obstacles" << std::endl;
			got_next=false;
		}else{
//			std::cout << "shortest path found" << std::endl;
//			for(int i=0;i<result->length();++i)
//			{
//				std::cout << m_nodes[result->GetVertex(i)->getID()].first;
//				std::cout << "->";
//			}
//			std::cout << std::endl <<  "*********************************************" << std::endl;
			int next_point_id=result->GetVertex(1)->getID();
			//do stuff to prepare next iteration
			//m_visited[m_current_coord]=	m_visited[m_current_coord]+1;

			m_misleading[m_current_coord].insert(m_nodes[next_point_id].first);

	//		std::cout<<"before adding element"<< std::endl;
	//		for(std::deque<octomap::point3d>::iterator it=m_current_path.begin(),end=m_current_path.end();it!=end;++it){
	//			std::cout<< (*it) << std::endl;
	//		}

			m_current_path.push_back(m_nodes[next_point_id].first);
			m_path_cost.push_back(m_cost[next_point_id]);

			int mv_fwd=2;
			while(result->length()>mv_fwd){
				int next_point_id2=result->GetVertex(mv_fwd)->getID();
				//TODO : repalce test by if next vertex at finest resolution, aka, no children
				if(m_nodes[next_point_id2].second==m_nodes[next_point_id].second){
					//m_visited[m_nodes[next_point_id].first]=	m_visited[m_nodes[next_point_id].first]+1;
					m_misleading[m_nodes[next_point_id].first].insert(m_nodes[next_point_id2].first);
					m_current_path.push_back(m_nodes[next_point_id2].first);
					m_path_cost.push_back(m_cost[next_point_id2]);
					next_point_id=next_point_id2;
					++mv_fwd;
				}else{
					break;
				}
			}

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
		}
	}
	if(!got_next){
		//std::cout << "shortest path not found" << std::endl;
		//go back // if path empty => no solution return false
		//m_visited[m_current_coord]=0;
		m_misleading[m_current_coord].clear();
		m_nb_backtrack++;
		m_current_path.pop_back();
		if(m_current_path.size()==0){
			//no possible path
			return false;
		}else{
			m_current_coord=m_current_path.back();
			m_path_cost.pop_back();
			return true;
		}
	}
}

bool MSP3D::run(){
	if(m_tree.search(m_start)->getOccupancy()>1-m_epsilon || m_tree.search(m_end)->getOccupancy()>1-m_epsilon){
		std::cout<<"start or end is an obstacle"<< std::endl;
		return false;
	}
	while(step()){/*std::cout<<*/++m_nb_step;}
//	std::stringstream it_name;
//	it_name << "iteration" << m_nb_step << ".ot";
//	kshortestpaths::BasePath result(std::vector<kshortestpaths::BaseVertex*>(),0);
//	visu(std::string(it_name.str()),&result);
	std::cout<< "NB backtrack : " << m_nb_backtrack << std::endl;
	if(m_path_found){
		return true;
	}else{
		return false;
	}
}


bool MSP3D::runAs(){
	clock_t tstart = clock();
	Gfull();
	clock_t tend1 = clock();
	kshortestpaths::DijkstraShortestPathAlg shortest_path_alg(&m_graph);
	kshortestpaths::BasePath* result =
		shortest_path_alg.get_shortest_path(
				m_graph.get_vertex(m_start_index), m_graph.get_vertex(m_end_index));
	clock_t tend2 = clock();
	std::cout << "Time to calculate GFull: " << (tend1-tstart)*1.0/CLOCKS_PER_SEC << std::endl;
	std::cout << "Time to calculate A*: " << (tend2-tend1)*1.0/CLOCKS_PER_SEC << std::endl;
	std::cout << "Total time: " << (tend2-tstart)*1.0/CLOCKS_PER_SEC << std::endl;
	std::cout <<  "start :" << m_start_index << ", end :" << m_end_index << std::endl;
//	double scale=m_tree.getResolution()*pow(2,16-m_max_tree_depth)/(1-8);
//	for(int i=0;i<result->length();++i)
//	{
//		//std::cout << m_nodes[result->GetVertex(i)->getID()].first;
//		std::cout << (((m_nodes[result->GetVertex(i)->getID()].first)*(1/scale))+octomap::point3d(3.5,3.5,3.5))*(1.0/7.0);
//		std::cout << "->";
//	}
//	std::cout << std::endl <<  "*********************************************" << std::endl;
	std:;cout << "Cost: " << result->Weight() << " Length: " << result->length() << std::endl;
}

std::deque<octomap::point3d> MSP3D::getPath(){return m_current_path;}

double MSP3D::getPathCost(){
	return std::accumulate(m_path_cost.begin(),m_path_cost.end(),0.0);//
//	double cost=0;
//	std::deque<octomap::point3d>::iterator it=m_current_path.begin(),end=m_current_path.end();
//	++it;
//	for(0;it!=end;++it){
//		cost += low_cost(*it);
////		std::cout << low_cost(*it) << " -> ";
//	}
////	std::cout<<std::endl;
//	return cost;
}

bool MSP3D::inPath(octomap::point3d pt,double size){
//	std::cout << "in path intro" << std::endl;
//	for(std::deque<octomap::point3d>::iterator it=m_current_path.begin(),end=m_current_path.end();it!=end;++it){
//				std::cout<< (*it) << std::endl;
//			}


	//TODO : change it to if it includes an element of the path, i e, if is_in(*it, pt, pt_size)

	for(std::deque<octomap::point3d>::iterator it=m_current_path.begin(),end=m_current_path.end();it!=end;++it){

//		std::cout << "in path test " <<pt << (*it) << std::endl;
		//if(pt==(*it) && !((*it)==m_current_path.back())){
		if(is_in(*it,std::pair<octomap::point3d,double>(pt,size)) && !((*it)==m_current_path.back())){
//			std::cout << "in path " << std::endl;
			return true;
		}
	}
	return false;
}

void MSP3D::add_node_to_reduced_vertices(octomap::OcTreeNode* node,octomap::point3d coord, double size){
//	std::cout << coord << " , " << size << " , " << node->getOccupancy() <<std::endl;
	if((coord-m_current_coord).norm()>m_alpha*size  || !(node->hasChildren())){
		if(!inPath(coord,size)
				&& node->getOccupancy()<1-m_epsilon
				&& m_current_forbidden.find(coord)==m_current_forbidden.end()
			){
			m_nodes.push_back(std::pair<octomap::point3d,double>(coord,size));
			m_cost.push_back(cost_func(node->getOccupancy())*pow(size/m_tree.getNodeSize(m_max_tree_depth),3));
		}
	}else{
		for(int i=0;i<8;++i){
			if(size<10.0 && !node->childExists(i)){
				node->createChild(i);
			}
			if(node->childExists(i)){
				add_node_to_reduced_vertices(node->getChild(i),coord+m_child_dir[i]*0.25*size,size*0.5);
			}
		}
	}
}

void MSP3D::reducedGraph(){
	m_graph.clear();
	m_nodes.clear();
	m_cost.clear();
	m_start_index=-1;
	m_end_index=-1;
	try {
		m_current_forbidden=m_misleading.at(m_current_coord);
	}catch (const std::out_of_range& oor) {
		m_current_forbidden=std::set<octomap::point3d,Point3D_Less>();
	  }
	add_node_to_reduced_vertices(m_tree.getRoot(),octomap::point3d(0,0,0),m_tree.getNodeSize(0));

	int l=m_nodes.size();

//	std::cout<< "number of nodes: " << l << std::endl;
	for(int i=0;i<l;++i){
		// !!!!!!!!!!!!!  if not in path?
//		std::cout<< "node " << i << ":" << m_nodes[i].first <<std::endl;
		m_graph.add_vertex(i,m_lambda2*(m_nodes[i].first-m_end_coord).norm());
		if(is_start(m_nodes[i])){
//			std::cout<<"start: "<< m_nodes[i].first <<std::endl;
			if(m_start_index!=-1){
				std::cout << "2 start nodes, fail" << std::endl;
				return;
				//exit(1);
			}
			m_start_index=i;
		}
		if(is_goal(m_nodes[i])){
//			std::cout<<"end: "<< m_nodes[i].first <<std::endl;
			if(m_end_index!=-1){
				std::cout << "2 end nodes, fail" << std::endl;
				return;
				//exit(1);
			}
			m_end_index=i;
		}
	}
	if(m_start_index==-1){
		std::cout << "0 start node, fail" << std::endl;
		return;
	}
	if(m_end_index==-1){
		std::cout << "0 end node, fail" << std::endl;
		return;
	}
	for(int i=0;i<l;++i){
		for(int j=i+1;j<l;++j){
				if(neighboor(m_nodes[i],m_nodes[j])){
//					std::cout<< "neighboor:" << i << "," << j <<std::endl;
//					std::cout<< "cost:" << cost(i,j) <<std::endl;
					m_graph.add_edge(i,j,m_cost[j]);
					m_graph.add_edge(j,i,m_cost[i]);
				}
		}
	}
}


void MSP3D::Gfull(){
	m_graph.clear();
	m_nodes.clear();
	m_cost.clear();
	m_start_index=-1;
	m_end_index=-1;
	m_current_coord=m_start_coord;
	//Calcul of Gfull
	for(octomap::OcTree::leaf_iterator it = m_tree.begin_leafs(),	end=m_tree.end_leafs(); it!= end; ++it){
		if(it->getOccupancy()<1-m_epsilon){
			m_nodes.push_back(std::pair<octomap::point3d,double>(it.getCoordinate(),it.getSize()));
			m_cost.push_back(cost_func(it->getOccupancy())*pow(it.getSize()/m_tree.getNodeSize(m_max_tree_depth),3));
		}
	}
	int l=m_nodes.size();

	std::cout<< "number of nodes: " << l << std::endl;
	for(int i=0;i<l;++i){
		// !!!!!!!!!!!!!  if not in path?
//		std::cout<< "node " << i << ":" << m_nodes[i].first <<std::endl;
		m_graph.add_vertex(i,m_lambda2*(m_nodes[i].first-m_end_coord).norm());
		if(is_start(m_nodes[i])){
			std::cout<<"start: "<< m_nodes[i].first <<std::endl;
			if(m_start_index!=-1){
				std::cout << "2 start nodes, fail" << std::endl;
				//exit(1);
			}
			m_start_index=i;
		}
		if(is_goal(m_nodes[i])){
			std::cout<<"end: "<< m_nodes[i].first <<std::endl;
			if(m_end_index!=-1){
				std::cout << "2 end nodes, fail" << std::endl;
				//exit(1);
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
		double progress=i*1.0/l;
		int barWidth = 70;

		std::cout << "[";
		int pos = barWidth * progress;
		for (int j = 0; j < barWidth; ++j) {
			if (j < pos) std::cout << "=";
			else if (j == pos) std::cout << ">";
			else std::cout << " ";
		}
		std::cout << "] " << int(progress * 100.0) << " %\r";
		std::cout.flush();
		for(int j=i+1;j<l;++j){
				if(neighboor(m_nodes[i],m_nodes[j])){
//					std::cout<< "neighboor:" << i << "," << j <<std::endl;
//					std::cout<< "cost:" << cost(i,j) <<std::endl;
					m_graph.add_edge(i,j,m_cost[j]);
					m_graph.add_edge(j,i,m_cost[i]);
				}
		}
	}
	std::cout << std::endl;
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
	return NULL;
}

void MSP3D::copyNode(octomap::OcTreeNode* n,octomap::OcTreeNode* nc){
	nc->setLogOdds(n->getLogOdds());
	if(n->hasChildren()){
		for(int i=0;i<8;++i){
			if(n->childExists(i)){
				if(!(nc->childExists(i))){
					nc->createChild(i);
				}
				copyNode(n->getChild(i),nc->getChild(i));
			}
		}
	}
}

void MSP3D::visu(std::string filename, kshortestpaths::BasePath* path){
	octomap::ColorOcTree tree(m_tree.getResolution());  // create empty tree with resolution 0.1
	tree.updateNode(0,0,0,false);
	octomap::ColorOcTreeNode* it=tree.getRoot();
	for(int i=0;i<8;++i){
		if(it->childExists(i)){
			it->deleteChild(i);
		}
	}
//	std::cout<< "root: " << tree.getRoot() << std::endl;
//	std::cout<< "root: " << m_tree.getRoot() << std::endl;
	copyNode(m_tree.getRoot(),tree.getRoot());
//	int mm=32;
//	for (int x=-mm; x<mm; x++) {
//		for (int y=-mm; y<mm; y++) {
//			for (int z=-mm; z<mm; z++) {
//				octomap::point3d endpoint ((float) x*100.0f, (float) y*100.0f, (float) z*100.0f);
//				tree.updateNode(endpoint, false); // integrate 'free' measurement
//			}
//		}
//	}
//
//	for(octomap::ColorOcTree::tree_iterator it = tree.begin_tree(),	end=tree.end_tree(); it!= end; ++it)
//	{
//		if(it.getDepth()>=m_max_tree_depth){
//			for(int i=0;i<8;++i){
//				if(it->childExists(i)){
//					it->deleteChild(i);
//				}
//			}
//			//octomap::point3d vec_dir(1,1,1);
//			it->setLogOdds(octomap::logodds(0));
//			//it->setLogOdds(octomap::logodds((vec_dir.cross(it.getCoordinate())).norm()/max_size));
//		}
//	}
//	tree.updateInnerOccupancy();

	octomap::ColorOcTreeNode* n;
//	std::cout << "starting to create color tree" << std::endl;
	for(octomap::ColorOcTree::tree_iterator it = tree.begin_tree(),	end=tree.end_tree(); it!= end; ++it)
	{
		for(int i=0;i<m_nodes.size();++i){
			if(it.getCoordinate()==m_nodes[i].first){
//				std::cout << "change occupancy of node" << findNode(it.getCoordinate())->getOccupancy() << std::endl;
				//it->setColor(0,0,(char)floor(125+125*(findNode(it.getCoordinate())->getOccupancy())));
				it->setColor(0,0,255);
				//it->setLogOdds(findNode(it.getCoordinate())->getLogOdds());
				it->setLogOdds(octomap::logodds(1));
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

void MSP3D::visu_init(std::string filename){

	octomap::ColorOcTree tree(m_tree.getResolution());
	tree.updateNode(0,0,0,false);
	octomap::ColorOcTreeNode* it=tree.getRoot();
	for(int i=0;i<8;++i){
		if(it->childExists(i)){
			it->deleteChild(i);
		}
	}
//	it->pruneNode();

	copyNode(m_tree.getRoot(),tree.getRoot());

	for(octomap::ColorOcTree::tree_iterator it = tree.begin_tree(),	end=tree.end_tree(); it!= end; ++it)
	{
//		if(it.getDepth()>=m_max_tree_depth){
//			for(int i=0;i<8;++i){
//				if(it->childExists(i)){
//					it->deleteChild(i);
//				}
//			}
//			//octomap::point3d vec_dir(1,1,1);
//			//it->setLogOdds(octomap::logodds((vec_dir.cross(it.getCoordinate())).norm()/max_size));
//		}

		it->setLogOdds(octomap::logodds(1));
	}
	tree.updateInnerOccupancy();


//	for(octomap::ColorOcTree::tree_iterator it = tree.begin_tree(),	end=tree.end_tree(); it!= end; ++it)
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

	int z=1;
	//previous path
	binary_outfile /*<< std::endl << "ppath" */<< z;

	m_start_coord.writeBinary(binary_outfile);
//	std::cout << std::endl << "ppath" << m_current_path.size();
//	for(std::deque<octomap::point3d>::iterator it=m_current_path.begin(),end=m_current_path.end();it!=end;++it){
//		it->writeBinary(binary_outfile);
////		std::cout << *it <<std::endl;
//	}

	//future path
	binary_outfile /*<< std::endl << "fpath"*/ << z;
	m_start_coord.writeBinary(binary_outfile);
	binary_outfile << m_tree.getNodeSize(m_max_tree_depth);
//	std::cout << std::endl << "fpath" << m_current_path.size();
//	for(int i=0;i<path->length();++i){
//		m_nodes[path->GetVertex(i)->getID()].first.writeBinary(binary_outfile);
//		binary_outfile << m_nodes[path->GetVertex(i)->getID()].second;
////		std::cout << m_nodes[path->GetVertex(i)->getID()].first <<std::endl;
//	}

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

//	binary_outfile /*<< std::endl << "fpath"*/ << z;
//	m_start_coord.writeBinary(binary_outfile);
//	binary_outfile << m_tree.getNodeSize(m_max_tree_depth);


	binary_outfile.close();

	//exit(0);
}

bool MSP3D::is_start(std::pair<octomap::point3d,double> &node){
	return is_in(m_current_coord,node);
}

bool MSP3D::is_goal(std::pair<octomap::point3d,double> &node){
	return is_in(m_end_coord,node);
}

bool MSP3D::is_in(octomap::point3d pt,std::pair<octomap::point3d,double> node){
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
	return cost_func(findNode(pt)->getOccupancy());
	//std::cout << pt << F << std::endl;

}

double MSP3D::cost_func(double F){
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
