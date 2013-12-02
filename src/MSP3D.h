
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

namespace msp{
	class MSP3D{
		public:
			MSP3D(octomap::OcTree &tree);
			bool init(octomap::point3d start,octomap::point3d end);
			bool step();
			bool run();
			void getPath();
		protected:
			void tree2nodes();
			void nodesAdjacency();
			void kthShortestPath();
	};
}
