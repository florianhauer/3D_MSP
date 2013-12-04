
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <deque>
#include <map>
#include <vector>
#include <utility>
#include "Graph.h"

namespace msp{
	class MSP3D{
		public:
			MSP3D(octomap::OcTree &tree);
			bool init(octomap::point3d start,octomap::point3d end);
			bool step();
			bool run();
			std::deque<octomap::OcTreeKey> getPath();
		protected:
			void reducedGraph();
			void iterativeReducedGraph(octomap::OcTree::NodeType *n);
			bool neighboor(std::pair<octomap::OcTreeKey,double> &na,std::pair<octomap::OcTreeKey,double> &nb);
			void kthShortestPath();
			octomap::OcTreeKey m_start;
			octomap::OcTreeKey m_end;
			octomap::OcTreeKey m_current_point;
			octomap::point3d m_current_coord;
			octomap::OcTree m_tree;
			bool m_path_found;
			std::deque<octomap::OcTreeKey> m_current_path;
			kshortestpaths::Graph m_graph;
			std::map<octomap::OcTreeKey,double> m_visited;
			double m_alpha;//used in reduced graph as parameter for decomposition
			std::vector<std::pair<octomap::OcTreeKey,double> > m_nodes;
	};
}
