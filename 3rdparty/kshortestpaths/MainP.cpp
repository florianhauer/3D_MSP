/************************************************************************/
/* $Id: MainP.cpp 65 2010-09-08 06:48:36Z yan.qi.asu $                                                                 */
/************************************************************************/

#include <limits>
#include <set>
#include <map>
#include <queue>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <algorithm>
#include "GraphElements.h"
#include "Graph.h"
#include "DijkstraShortestPathAlg.h"
#include "YenTopKShortestPathsAlg.h"

using namespace std;
using namespace kshortestpaths;


void testDijkstraGraph()
{
	Graph* my_graph_pt = new Graph("../data/test_1");
	DijkstraShortestPathAlg shortest_path_alg(my_graph_pt);
	BasePath* result =
		shortest_path_alg.get_shortest_path(
			my_graph_pt->get_vertex(0), my_graph_pt->get_vertex(5));
	result->PrintOut(cout);
}

void testDijkstraGraphDisconnectedGraph()
{
	cout << "Test disconnected graph" << endl;
	Graph graph;
	graph.add_vertex(0);
	graph.add_vertex(1);
	DijkstraShortestPathAlg shortest_path_alg(&graph);
	BasePath* result =
		shortest_path_alg.get_shortest_path(
				graph.get_vertex(0), graph.get_vertex(1));
	result->PrintOut(cout);
}

void testYenAlg()
{
	Graph my_graph("/home/florian/workspace/3D_MSP/3rdparty/kshortestpaths/data/test_1");

	YenTopKShortestPathsAlg yenAlg(my_graph, my_graph.get_vertex(0),
		my_graph.get_vertex(5));

	int i=0;
	while(yenAlg.has_next())
	{
		++i;
		yenAlg.next()->PrintOut(cout);
	}

// 	System.out.println("Result # :"+i);
// 	System.out.println("Candidate # :"+yenAlg.get_cadidate_size());
// 	System.out.println("All generated : "+yenAlg.get_generated_path_size());

}
void testYenAlg2()
{
	kshortestpaths::Graph my_graph;
	for(int i=0;i<8;++i){
		my_graph.add_vertex(i);
	}
	for(int i=0;i<5;++i){
		my_graph.add_edge(i,i+1,0.4);
		my_graph.add_edge(i,i+2,0.4);
		my_graph.add_edge(i,i+3,0.4);
	}
	my_graph.add_edge(5,5+1,0.4);
	my_graph.add_edge(5,5+2,0.4);
	my_graph.add_edge(5,0,0.4);
	my_graph.add_edge(6,6+1,0.4);
	my_graph.add_edge(6,0,0.4);
	my_graph.add_edge(6,1,0.4);
	my_graph.add_edge(7,0,0.4);
	my_graph.add_edge(7,1,0.4);
	my_graph.add_edge(7,2,0.4);

	YenTopKShortestPathsAlg yenAlg(my_graph, my_graph.get_vertex(6),
		my_graph.get_vertex(7));

	int i=0;
	while(yenAlg.has_next())
	{
		++i;
		yenAlg.next()->PrintOut(cout);
	}

// 	System.out.println("Result # :"+i);
// 	System.out.println("Candidate # :"+yenAlg.get_cadidate_size());
// 	System.out.println("All generated : "+yenAlg.get_generated_path_size());

}

int main(...)
{
//	cout << "Welcome to the real world!" << endl;

	//testDijkstraGraph();
//	cout << "Yen" << endl;
//	testYenAlg();
	cout << "Yen 2" << endl;
	testYenAlg2();
//	testDijkstraGraphDisconnectedGraph();
}
