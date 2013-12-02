#include "MSP3D.h"


namespace msp{
MSP3D::MSP3D(octomap::OcTree &tree){}
bool MSP3D::init(octomap::point3d start,octomap::point3d end){return true;}
bool MSP3D::step(){return true;}
bool MSP3D::run(){return true;}
void MSP3D::getPath(){}
void MSP3D::tree2nodes(){}
void MSP3D::nodesAdjacency(){}
void MSP3D::kthShortestPath(){}
}
