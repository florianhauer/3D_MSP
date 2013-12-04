
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <deque>
#include <map>
#include <vector>
#include <utility>
#include "Graph.h"

class Point3D_Less :
public binary_function<octomap::point3d, octomap::point3d, bool> {
	public :
	bool operator()(const octomap::point3d &lhs, const octomap::point3d& rhs) const{
    	if(lhs.x()<rhs.x())
			return 1;
		if(lhs.x()>rhs.x())
			return 0;
    	if(lhs.y()<rhs.y())
			return 1;
		if(lhs.y()>rhs.y())
			return 0;
    	if(lhs.z()<rhs.z())
			return 1;
    	else
    		return 0;
    }
};

namespace msp{
	class MSP3D{
		public:
			MSP3D(octomap::OcTree &tree);
			bool init(octomap::point3d start,octomap::point3d end);
			bool step();
			bool run();
			std::deque<octomap::point3d> getPath();
		protected:
			void reducedGraph();
			void iterativeReducedGraph(octomap::OcTree::NodeType *n);
			bool neighboor(std::pair<octomap::point3d,double> &na,std::pair<octomap::point3d,double> &nb);
			bool is_start(std::pair<octomap::point3d,double> &node);
			bool is_goal(std::pair<octomap::point3d,double> &node);
			bool is_in(octomap::point3d pt,std::pair<octomap::point3d,double> &node);
			double cost(int i, int j);
			octomap::OcTreeKey m_start;
			octomap::OcTreeKey m_end;
			octomap::point3d m_start_coord;
			octomap::point3d m_end_coord;
			//octomap::OcTreeKey m_current_point;
			octomap::point3d m_current_coord;
			octomap::OcTree m_tree;
			int m_start_index;
			int m_end_index;
			bool m_path_found;
			std::deque<octomap::point3d> m_current_path;
			kshortestpaths::Graph m_graph;
			std::map<octomap::point3d,double,Point3D_Less> m_visited;
			double m_alpha;//used in reduced graph as parameter for decomposition
			std::vector<std::pair<octomap::point3d,double> > m_nodes; //coord,size
			double m_eps;//used as margin for comparison (small compared to tree resolution)
			double m_M;
			double m_lambda1;
			double m_lambda2;
	};
}
