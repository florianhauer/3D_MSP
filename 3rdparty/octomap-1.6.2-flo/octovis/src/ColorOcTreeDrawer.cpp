/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License (octovis): GNU GPL v2
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <octovis/ColorOcTreeDrawer.h>


#define OTD_RAD2DEG 57.2957795

namespace octomap {

	ColorOcTreeDrawer::ColorOcTreeDrawer()
	  : OcTreeDrawer(),m_ptrajArraySize(0),m_ftrajArraySize(0),m_SEArraySize(0) {
		m_ptrajArray=NULL;
		m_ftrajArray=NULL;
		m_SEArray=NULL;
		m_obstaclesArray=NULL;
		m_ptrajArrayColor=NULL;
		m_ftrajArrayColor=NULL;
		m_SEArrayColor=NULL;
		m_obstaclesArrayColor=NULL;
	}

	ColorOcTreeDrawer::ColorOcTreeDrawer(std::deque<octomap::point3d> ppath,std::deque<std::pair<octomap::point3d,double> > fpath,octomap::point3d start,octomap::point3d end,std::deque<std::pair<octomap::point3d,double> > obstacles)
	  : OcTreeDrawer(),m_ppath(ppath),m_fpath(fpath),m_start(start),m_end(end),m_obstacles(obstacles) {
	}

  ColorOcTreeDrawer::~ColorOcTreeDrawer() {
  }

  void ColorOcTreeDrawer::clear() {
      //clearOcTree();

      clearCubes(&m_SEArray, m_SEArraySize, &m_SEArrayColor);
      clearCubes(&m_ptrajArray, m_ptrajArraySize, &m_ptrajArrayColor);
      clearCubes(&m_ftrajArray, m_ftrajArraySize, &m_ftrajArrayColor);
      clearCubes(&m_obstaclesArray, m_obstaclesArraySize, &m_obstaclesArrayColor);
      OcTreeDrawer::clear();
    }

  void ColorOcTreeDrawer::set(std::deque<octomap::point3d> ppath,std::deque<std::pair<octomap::point3d,double> > fpath,octomap::point3d start,octomap::point3d end,std::deque<std::pair<octomap::point3d,double> > obstacles){
		clearCubes(&m_SEArray, m_SEArraySize, &m_SEArrayColor);
		clearCubes(&m_ptrajArray, m_ptrajArraySize, &m_ptrajArrayColor);
		clearCubes(&m_ftrajArray, m_ftrajArraySize, &m_ftrajArrayColor);
		clearCubes(&m_obstaclesArray, m_obstaclesArraySize, &m_obstaclesArrayColor);
		m_ppath=ppath;
		m_fpath=fpath;
		m_start=start;
		m_end=end;
		m_obstacles=obstacles;
  }

  void ColorOcTreeDrawer::draw() const {

      // push current status
      glPushMatrix();
      // octomap::pose6d relative_transform = origin * initial_origin.inv();

      octomap::pose6d relative_transform = origin;// * initial_origin;

      // apply relative transform
      const octomath::Quaternion& q = relative_transform.rot();
      glTranslatef(relative_transform.x(), relative_transform.y(), relative_transform.z());

      // convert quaternion to angle/axis notation
      float scale = sqrt(q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
      if (scale) {
        float axis_x = q.x() / scale;
        float axis_y = q.y() / scale;
        float axis_z = q.z() / scale;
        float angle = acos(q.u()) * 2.0f * OTD_RAD2DEG;  //  opengl expects DEG
        glRotatef(angle, axis_x, axis_y, axis_z);
      }

      glEnableClientState(GL_VERTEX_ARRAY);


      drawOctreeGrid();
      drawPPath();
//    drawOccupiedVoxels();

//      if (m_drawOccupied)
//        drawOccupiedVoxels();
//      if (m_drawFree)
//        drawFreeVoxels();
//      if (m_drawOcTreeGrid)
//        drawOctreeGrid();
//      if (m_drawSelection)
//        drawSelection();


//      if (m_displayAxes) {
//        drawAxes();
//      }

      glDisableClientState(GL_VERTEX_ARRAY);

      // reset previous status
      glPopMatrix();

    }

  void ColorOcTreeDrawer::drawPPath() const {

	  glColor3f(0.1f, 0.1f, 0.1f);
//	  std::cout << "traj ploting size: " << m_ftrajArraySize << std::endl;
//	  std::cout << "m_traj  size: " << m_fpath.size() << std::endl;
      drawCubes(m_obstaclesArray, m_obstaclesArraySize, m_obstaclesArrayColor);
      drawCubes(m_SEArray, m_SEArraySize, m_SEArrayColor);
      drawCubes(m_ptrajArray, m_ptrajArraySize, m_ptrajArrayColor);
      drawCubes(m_ftrajArray, m_ftrajArraySize, m_ftrajArrayColor);

//      if (m_colorMode == CM_SEMANTIC) {
//        // hardcoded mapping id -> colors
//        if (this->map_id == 0) {  // background
//          glColor3f(0.784f, 0.66f, 0); // gold
//        }
//        else if (this->map_id == 1) {  // table
//          glColor3f(0.68f, 0., 0.62f); // purple
//        }
//        else { // object
//          glColor3f(0., 0.784f, 0.725f); // cyan
//        }
//        drawCubes(m_occupiedThresArray, m_occupiedThresSize, m_occupiedThresColorArray);
//      }
//      else {
//        // colors for printout mode:
//        if (m_colorMode == CM_PRINTOUT) {
//          if (!m_drawFree) { // gray on white background
//            glColor3f(0.6f, 0.6f, 0.6f);
//          }
//          else {
//            glColor3f(0.1f, 0.1f, 0.1f);
//          }
//        }
//
//        // draw binary occupied cells
//        if (m_occupiedThresSize != 0) {
//          if (m_colorMode != CM_PRINTOUT) glColor4f(0.0f, 0.0f, 1.0f, m_alphaOccupied);
//          drawCubes(m_occupiedThresArray, m_occupiedThresSize, m_occupiedThresColorArray);
//        }
//
//        // draw delta occupied cells
//        if (m_occupiedSize != 0) {
//          if (m_colorMode != CM_PRINTOUT) glColor4f(0.2f, 0.7f, 1.0f, m_alphaOccupied);
//          drawCubes(m_occupiedArray, m_occupiedSize, m_occupiedColorArray);
//        }
//      }
    }

  void ColorOcTreeDrawer::setOcTree(const AbstractOcTree& tree_pnt,
                                    const octomap::pose6d& origin_,
                                    int map_id_) {

    const ColorOcTree& tree = ((const ColorOcTree&) tree_pnt);

    this->map_id = map_id_;

    // save origin used during cube generation
    this->initial_origin = octomap::pose6d(octomap::point3d(0,0,0), origin_.rot());
    // origin is in global coords
    this->origin = origin_;
    
    // maximum size to prevent crashes on large maps: (should be checked in a better way than a constant)
    bool showAll = (tree.size() < 5 * 1e6);
    bool uses_origin = ( (origin_.rot().x() != 0.) && (origin_.rot().y() != 0.)
                         && (origin_.rot().z() != 0.) && (origin_.rot().u() != 1.) );

    // walk the tree one to find the number of nodes in each category
    // (this is used to set up the OpenGL arrays)
    // TODO: this step may be left out, if we maintained the GLArrays in std::vectors instead...
    unsigned int cnt_occupied(0), cnt_occupied_thres(0), cnt_free(0), cnt_free_thres(0);
    for(ColorOcTree::tree_iterator it = tree.begin_tree(this->m_max_tree_depth),
          end=tree.end_tree(); it!= end; ++it) {
      if (it.isLeaf()) { 
        if (tree.isNodeOccupied(*it)){ // occupied voxels
          if (tree.isNodeAtThreshold(*it)) ++cnt_occupied_thres;
          else                               ++cnt_occupied;
        }
        else if (showAll) { // freespace voxels
          if (tree.isNodeAtThreshold(*it)) ++cnt_free_thres;
          else                               ++cnt_free;
        }
      }        
    }    
    // setup GL arrays for cube quads and cube colors
    initGLArrays(cnt_occupied      , m_occupiedSize     , &m_occupiedArray     , &m_occupiedColorArray);
    initGLArrays(cnt_occupied_thres, m_occupiedThresSize, &m_occupiedThresArray, &m_occupiedThresColorArray);
    initGLArrays(cnt_free          , m_freeSize         , &m_freeArray, NULL);
    initGLArrays(cnt_free_thres    , m_freeThresSize    , &m_freeThresArray, NULL);

    initGLArrays(m_ppath.size()      , m_ptrajArraySize     , &m_ptrajArray     , &m_ptrajArrayColor);
    initGLArrays(m_fpath.size()      , m_ftrajArraySize     , &m_ftrajArray     , &m_ftrajArrayColor);
    initGLArrays(m_obstacles.size()      , m_obstaclesArraySize     , &m_obstaclesArray     , &m_obstaclesArrayColor);
    initGLArrays(2      , m_SEArraySize     , &m_SEArray     , &m_SEArrayColor);

    std::vector<octomath::Vector3> cube_template;
    initCubeTemplate(origin, cube_template);

    unsigned int idx_occupied(0), idx_occupied_thres(0), idx_free(0), idx_free_thres(0);
    unsigned int color_idx_occupied(0), color_idx_occupied_thres(0);

    unsigned int idx_ppath(0), color_idx_ppath(0);
    unsigned int idx_fpath(0), color_idx_fpath(0);
    unsigned int idx_obstacles(0), color_idx_obstacles(0);
    unsigned int idx_SE(0), color_idx_SE(0);

    m_grid_voxels.clear();
    OcTreeVolume voxel; // current voxel, possibly transformed 

    for(std::deque<octomap::point3d >::iterator it=m_ppath.begin(),end=m_ppath.end();it!=end;++it){
		//std::cout << "adding cube to plot traj " << it->first << std::endl;
		if (uses_origin)
		  voxel = OcTreeVolume(origin.rot().rotate(*it), m_fpath.front().second);
		else
		  voxel = OcTreeVolume(*it, m_fpath.front().second);
		idx_ppath = generateCube(voxel, cube_template, idx_ppath, &m_ptrajArray);
		color_idx_ppath =  setCubeColorRGBA(10, 10, 10,
											 (unsigned char) (255.),
											 color_idx_ppath, &m_ptrajArrayColor);
		// grid structure voxel
		//if (showAll) m_grid_voxels.push_back(voxel);
	}
    for(std::deque<std::pair<octomap::point3d,double> >::iterator it=m_fpath.begin(),end=m_fpath.end();it!=end;++it){
//		std::cout << "adding cube to plot traj " << it->first << std::endl;
		if (uses_origin)
		  voxel = OcTreeVolume(origin.rot().rotate(it->first), it->second);
		else
		  voxel = OcTreeVolume(it->first, it->second);
		idx_fpath = generateCube(voxel, cube_template, idx_fpath, &m_ftrajArray);
		color_idx_fpath =  setCubeColorRGBA(100, 100, 0,
											 (unsigned char) (200.),
											 color_idx_fpath, &m_ftrajArrayColor);
		// grid structure voxel
		//if (showAll) m_grid_voxels.push_back(voxel);
	}
    for(std::deque<std::pair<octomap::point3d,double> >::iterator it=m_obstacles.begin(),end=m_obstacles.end();it!=end;++it){
//		std::cout << "adding cube to plot obstacles " << it->first << std::endl;
		if (uses_origin)
		  voxel = OcTreeVolume(origin.rot().rotate(it->first), it->second);
		else
		  voxel = OcTreeVolume(it->first, it->second);
		idx_obstacles = generateCube(voxel, cube_template, idx_obstacles, &m_obstaclesArray);
		color_idx_obstacles =  setCubeColorRGBA(255, 140, 0,
											 (unsigned char) (200.),
											 color_idx_obstacles, &m_obstaclesArrayColor);
		// grid structure voxel
		//if (showAll) m_grid_voxels.push_back(voxel);
	}
    if (uses_origin)
	  voxel = OcTreeVolume(origin.rot().rotate(m_start), m_fpath.front().second+0.05);
	else
	  voxel = OcTreeVolume(m_start, m_fpath.front().second+0.05);
    idx_SE = generateCube(voxel, cube_template, idx_SE, &m_SEArray);
    color_idx_SE =  setCubeColorRGBA(0, 255, 0,
										 (unsigned char) (255.),
										 color_idx_SE, &m_SEArrayColor);
    if (uses_origin)
	  voxel = OcTreeVolume(origin.rot().rotate(m_end), m_fpath.front().second+0.05);
	else
	  voxel = OcTreeVolume(m_end, m_fpath.front().second+0.05);
    idx_SE = generateCube(voxel, cube_template, idx_SE, &m_SEArray);
    color_idx_SE =  setCubeColorRGBA(255, 0, 0,
										 (unsigned char) (255.),
										 color_idx_SE, &m_SEArrayColor);

    for(ColorOcTree::tree_iterator it = tree.begin_tree(this->m_max_tree_depth),
          end=tree.end_tree(); it!= end; ++it) {

      if (it.isLeaf()) { // voxels for leaf nodes
        if (uses_origin) 
          voxel = OcTreeVolume(origin.rot().rotate(it.getCoordinate()), it.getSize());
        else 
          voxel = OcTreeVolume(it.getCoordinate(), it.getSize());
        
        if (tree.isNodeOccupied(*it)){ // occupied voxels
          if (tree.isNodeAtThreshold(*it)) {
              idx_occupied_thres = generateCube(voxel, cube_template, idx_occupied_thres, &m_occupiedThresArray);
              color_idx_occupied_thres =  setCubeColorRGBA(it->getColor().r, it->getColor().g, it->getColor().b,
                                                           (unsigned char) (0.+it->getOccupancy() * 100.),
                                                           color_idx_occupied_thres, &m_occupiedThresColorArray);

//              idx_occupied_thres = generateCube(voxel, cube_template, idx_occupied_thres, &m_occupiedThresArray);
//              color_idx_occupied_thres =  setCubeColorRGBA(255, 255, 255,
//                                                           (unsigned char) ( 100.),
//                                                           color_idx_occupied_thres, &m_occupiedThresColorArray);
            }
          else {
              idx_occupied = generateCube(voxel, cube_template, idx_occupied, &m_occupiedArray);
              color_idx_occupied = setCubeColorRGBA(it->getColor().r, it->getColor().g, it->getColor().b,
                                                    (unsigned char)(0.+it->getOccupancy() * 100.),
                                                    color_idx_occupied, &m_occupiedColorArray);

//              idx_occupied = generateCube(voxel, cube_template, idx_occupied, &m_occupiedArray);
//              color_idx_occupied = setCubeColorRGBA(255,255,255,
//                                                    (unsigned char)(100.),
//                                                    color_idx_occupied, &m_occupiedColorArray);
            }
        }
        else if (showAll) { // freespace voxels
          if (tree.isNodeAtThreshold(*it)) {
            idx_free_thres = generateCube(voxel, cube_template, idx_free_thres, &m_freeThresArray);
          }
          else {
            idx_free = generateCube(voxel, cube_template, idx_free, &m_freeArray);
          }
        }

        // grid structure voxel
        if (showAll) m_grid_voxels.push_back(voxel);        
      }
      
      else { // inner node voxels (for grid structure only)
        if (showAll) {
          if (uses_origin)
            voxel = OcTreeVolume(origin.rot().rotate(it.getCoordinate()), it.getSize());
          else
            voxel = OcTreeVolume(it.getCoordinate(), it.getSize());
          m_grid_voxels.push_back(voxel);
        }
      }      
    } // end for all voxels

    m_octree_grid_vis_initialized = false;

    if(m_drawOcTreeGrid)
      initOctreeGridVis();    
  }

} // end namespace
