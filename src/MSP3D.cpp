#include "MSP3D.h"
#include "YenTopKShortestPathsAlg.h"


namespace msp{
MSP3D::MSP3D(octomap::OcTree &tree):m_tree(tree),m_path_found(false),m_alpha(1),m_eps(tree.getResolution()/10.0) {}

bool MSP3D::init(octomap::point3d start,octomap::point3d end){
	if(m_tree.coordToKeyChecked(start,m_start) && m_tree.coordToKeyChecked(end,m_end)){
		//m_current_point=m_start;
		m_current_coord=start;
		m_start_coord=start;
		m_end_coord=end;
		return true;
	}
	return false;
}

bool MSP3D::step(){
	reducedGraph();
	// shortest path
	kshortestpaths::YenTopKShortestPathsAlg yenAlg(m_graph, m_graph.get_vertex(m_start_index),m_graph.get_vertex(m_end_index));
	bool got_next=false;
	if(m_visited.count(m_current_coord)==0){
		if(yenAlg.has_next()){
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
	//if solution
	if(got_next){
		//go forward // if goal return false;
		int next_point_id=yenAlg.next()->GetVertex(1)->getID();
		//do stuff to prepare next iteration
		m_visited[m_current_coord]=	m_visited[m_current_coord]+1;
		m_current_path.push_back(m_nodes[next_point_id].first);
		//m_current_point;
		m_current_coord=m_nodes[next_point_id].first;

		if(next_point_id==m_end_index){
			return false;
		}else{
			return true;
		}
	}else{
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
	while(step()){}
	if(m_path_found){
		return true;
	}else{
		return false;
	}
}

std::deque<octomap::point3d> MSP3D::getPath(){return m_current_path;}

void MSP3D::reducedGraph(){
	m_graph.clear();
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
				m_nodes.push_back(std::pair<octomap::point3d,double>(it.getCoordinate(),it.getSize()));
				skip=true;
				depth=it.getDepth();
			}
		}
	}
	int l=m_nodes.size();
	for(int i=0;i<l;++i){
		// !!!!!!!!!!!!!  if not in path?
		m_graph.add_vertex(i);
		if(is_start(m_nodes[i])){
			m_start_index=i;
		}
		if(is_goal(m_nodes[i])){
			m_end_index=i;
		}
		for(int j=i+1;j<l;++j){
				if(neighboor(m_nodes[i],m_nodes[j])){
					m_graph.add_edge(i,j,cost(i,j));
					m_graph.add_edge(j,i,cost(j,i));
				}
		}
	}
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
	//if cost(j) <= 1-epsilon
	// (lambda1 * cost(j) + lambda2)*Volume(j)
	//else
	// M
	return 0.0;
}

bool MSP3D::neighboor(std::pair<octomap::point3d,double> &na,std::pair<octomap::point3d,double> &nb){
	double l=0.5*(na.second+nb.second);
	int c=0;
	if(fabs(na.first.x()-nb.first.x())<=(l+m_eps) && fabs(na.first.y()-nb.first.y())<=(l+m_eps) && fabs(na.first.z()-nb.first.z())<=(l+m_eps)){
		if(fabs(na.first.x()-nb.first.x())<l){
			++c;
		}
		if(fabs(na.first.y()-nb.first.y())<l){
			++c;
		}
		if(fabs(na.first.z()-nb.first.z())<l){
			++c;
		}
		if(c==2){
			return true;
		}
	}
	return false;
}

}
