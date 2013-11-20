/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

using namespace std;
using namespace octomap;

std::string dec2bin(unsigned n){
    char result[(sizeof(unsigned)*8)+1];
    unsigned index=sizeof(unsigned)*8;
    result[index]='\0';
    do result[--index]='0'+(n&1);
    while (n>>=1);
    return std::string(result+index);
}

bool neighboor(const OcTree::tree_iterator &small,const OcTree::tree_iterator &big){

	/*OcTreeKey kkk;
	cout << "neighboor"<<endl;
	cout << "coord: " <<small.getCoordinate();
		cout  << " , size: "<< small.getSize();
		cout <<" , depth: "<< small.getDepth();
		kkk=small.getKey();
		cout <<" , key: "<< dec2bin(kkk[0]) << "," << dec2bin(kkk[1]) << "," << dec2bin(kkk[2]) <<endl;
	cout << "coord: " <<big.getCoordinate();
		cout  << " , size: "<< big.getSize();
		cout <<" , depth: "<< big.getDepth();
		kkk=big.getKey();
		cout <<" , key: "<< dec2bin(kkk[0]) << "," << dec2bin(kkk[1]) << "," << dec2bin(kkk[2]) <<endl;*/
	//cout<<"neighboor"<<endl;
	double l;
	//cout<<"neighboor10"<<endl;
	l=0.5*(small.getSize()+big.getSize());
	//cout<< "l:" << l << endl;
	//cout<<"neighboor11"<<endl;
	int c;
	c=0;
	//cout<<"neighboor2"<<endl;

		/*
		cout<<"X:"<<fabs(small.getX()-big.getX())-l<<endl;
		cout<<"Y:"<<fabs(small.getY()-big.getY())-l<<endl;
		cout<<"Z:"<<fabs(small.getZ()-big.getZ())-l<<endl;//*/

///ATTENTION REPLACE 0.001 BY SOME EPSILON SMALLER THAN THE TREE RESOLUTION
	if(fabs(small.getX()-big.getX())<=(l+0.001) && fabs(small.getY()-big.getY())<=(l+0.001) && fabs(small.getZ()-big.getZ())<=(l+0.001)){
		/*
		cout<<"neighboor3"<<endl;
		cout<<"X:"<<fabs(small.getX()-big.getX())<<endl;
		cout<<"Y:"<<fabs(small.getY()-big.getY())<<endl;
		cout<<"Z:"<<fabs(small.getZ()-big.getZ())<<endl;//*/
		if(fabs(small.getX()-big.getX())<l){
			//cout<<"X:"<<fabs(small.getX()-big.getX())<<endl;
			++c;
		}
		if(fabs(small.getY()-big.getY())<l){
			//cout<<"Y:"<<fabs(small.getY()-big.getY())<<endl;
			++c;
		}
		if(fabs(small.getZ()-big.getZ())<l){
			//cout<<"Z:"<<fabs(small.getZ()-big.getZ())<<endl;
			++c;
		}
		//if(c==3)
			//cout<<"inside"<<endl;
		if(c==2){
			//cout<< "YAYYAYYAYYAYYAYYAYYAYYAYYAYYAYYAYYAYYAYYAYYAYYAYYAYYAYYAYYAYYAYYAYYAYYAY"<<endl;
			//cout<<endl<<endl<<endl;
			return true;
		}
	}
	//cout<<"neighboor4"<<endl;
	//cout<<endl<<endl<<endl;
	return false;
}

void print_query_info(point3d query, OcTreeNode* node) {
  if (node != NULL) {
    cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
  }
  else 
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;    
}

int main(int argc, char** argv) {

  cout << endl;
  cout << "generating example map" << endl;

  OcTree tree (0.1);  // create empty tree with resolution 0.1

int mm=3;
  // insert some measurements of occupied cells

  for (int x=-mm; x<mm; x++) {
    for (int y=-mm; y<mm; y++) {
      for (int z=-mm; z<mm; z++) {
        point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
        tree.updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }

  // insert some measurements of free cells

//  for (int x=-mm; x<mm; x++) {
//    for (int y=-mm; y<mm; y++) {
//      for (int z=-mm; z<mm; z++) {
//        point3d endpoint ((float) x*0.02f-1.0f, (float) y*0.02f-1.0f, (float) z*0.02f-1.0f);
//        tree.updateNode(endpoint, false);  // integrate 'free' measurement
//      }
//    }
//  }

  cout << endl;
  cout << "performing some queries:" << endl;
  
  point3d query (0., 0., 0.);
  OcTreeNode* result = tree.search (query);
  print_query_info(query, result);

  query = point3d(-1.,-1.,-1.);
  result = tree.search (query);
  print_query_info(query, result);

  query = point3d(1.,1.,1.);
  result = tree.search (query);
  print_query_info(query, result);

OcTree::tree_iterator it=tree.begin_tree();
OcTree::tree_iterator end=tree.end_tree();
OcTreeKey kkk;

OcTree::tree_iterator leaf1;

int c=0;

for(it;it!=end;++it){
	/*if(it.isLeaf() && (leaf1==OcTree::tree_iterator())){
		leaf1=it;
		cout<< "leaf1 :"<<endl;
		cout << "coord: " <<it.getCoordinate();
		cout  << " , size: "<< it.getSize();
		cout <<" , depth: "<< it.getDepth();
		kkk=it.getKey();
		cout <<" , key: "<< dec2bin(kkk[0]) << "," << dec2bin(kkk[1]) << "," << dec2bin(kkk[2]) <<endl;
	}
	if(!(leaf1==OcTree::tree_iterator())  && neighboor(leaf1,it)){
		cout << "coord: " <<it.getCoordinate();
		cout  << " , size: "<< it.getSize();
		cout <<" , depth: "<< it.getDepth();
		kkk=it.getKey();
		cout <<" , key: "<< dec2bin(kkk[0]) << "," << dec2bin(kkk[1]) << "," << dec2bin(kkk[2]) <<endl;
	}
	++c;
	if(c>30)
		break;*/
	if(fabs(it.getX()-0.05)<0.001 && fabs(it.getY()-0.05)<0.001 && fabs(it.getZ()-0.05)<0.001){
		leaf1=it;
		cout<<"found leaf1"<<endl;
		cout << "coord: " <<it.getCoordinate();
		cout  << " , size: "<< it.getSize()<<endl;
		break;
	}
}
cout<<"not found leaf1?"<<endl;
cout << "coord: " <<leaf1.getCoordinate();
cout  << " , size: "<< leaf1.getSize()<<endl;

for(it=tree.begin_tree();it!=end;++it){
	if(neighboor(leaf1,it)){
		cout << "coord: " <<it.getCoordinate();
		cout  << " , size: "<< it.getSize()<<endl;
	}
}



  cout << endl;
  tree.writeBinary("simple_tree.bt");
  cout << "wrote example file simple_tree.bt" << endl << endl;
  cout << "now you can use octovis to visualize: octovis simple_tree.bt"  << endl;
  cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl;  

}
