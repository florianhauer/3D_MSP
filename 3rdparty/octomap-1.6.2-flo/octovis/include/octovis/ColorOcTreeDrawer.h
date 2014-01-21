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

#ifndef OCTOVIS_COLOR_OCTREEDRAWER_H_
#define OCTOVIS_COLOR_OCTREEDRAWER_H_

#include <octovis/OcTreeDrawer.h>
#include <octomap/ColorOcTree.h>
#include <deque>

namespace octomap {

  class ColorOcTreeDrawer : public OcTreeDrawer {
  public:
	    ColorOcTreeDrawer();
	    ColorOcTreeDrawer(std::deque<octomap::point3d> ppath,std::deque<std::pair<octomap::point3d,double> > fpath,octomap::point3d start,octomap::point3d end,std::deque<std::pair<octomap::point3d,double> > obstacles);
    virtual ~ColorOcTreeDrawer();

    void draw() const ;
    void clear() ;
    void set(std::deque<octomap::point3d> ppath,std::deque<std::pair<octomap::point3d,double> > fpath,octomap::point3d start,octomap::point3d end,std::deque<std::pair<octomap::point3d,double> > obstacles);

    void drawPPath() const ;

    virtual void setOcTree(const AbstractOcTree& tree_pnt, const pose6d& origin, int map_id_);

  protected:
    std::deque<octomap::point3d> m_ppath;
    std::deque<std::pair<octomap::point3d,double> > m_fpath;
    octomap::point3d m_start;
    octomap::point3d m_end;
    std::deque<std::pair<octomap::point3d,double> > m_obstacles;

    GLfloat** m_ptrajArray;
    unsigned int m_ptrajArraySize;
    GLfloat* m_ptrajArrayColor;

    GLfloat** m_ftrajArray;
    unsigned int m_ftrajArraySize;
    GLfloat* m_ftrajArrayColor;

    GLfloat** m_SEArray;
    unsigned int m_SEArraySize;
    GLfloat* m_SEArrayColor;

    GLfloat** m_obstaclesArray;
    unsigned int m_obstaclesArraySize;
    GLfloat* m_obstaclesArrayColor;

    
  };


} // end namespace

#endif
