/*
 *  abstractGraph.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 12/07/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

/*
 * The Vertex class
 */
template <class Abstract_Graph> class Abstract_Node 
{
public:
	Abstract_Node(Abstract_Graph& g_, const char name_[]) : g(g_),name(name_) { }
	
	// Vertex definition
	typedef typename boost::graph_traits<Abstract_Graph>::vertex_descriptor Vertex;
	
	void operator()(const Vertex& v) const
	{
		using namespace boost;
		typename property_map<Abstract_Graph, vertex_index_t>::type
		vertex_id = get(vertex_index, g);
		std::cout << "vertex: " << name[get(vertex_id, v)] << std::endl;
		
		// Write out the outgoing edges
		std::cout << "\tout-edges: ";
		typename graph_traits<Abstract_Graph>::out_edge_iterator out_i, out_end;
		typename graph_traits<Abstract_Graph>::edge_descriptor e;
		for (tie(out_i, out_end) = out_edges(v, g);
				 out_i != out_end; ++out_i)
		{
			e = *out_i;
			Vertex src = source(e, g), targ = target(e, g);
			std::cout << "(" << name[get(vertex_id, src)]
			<< "," << name[get(vertex_id, targ)] << ") ";
		}
		std::cout << std::endl;
		
		// Write out the incoming edges
		std::cout << "\tin-edges: ";
		typename graph_traits<Abstract_Graph>::in_edge_iterator in_i, in_end;
		for (tie(in_i, in_end) = in_edges(v, g); in_i != in_end; ++in_i)
		{
			e = *in_i;
			Vertex src = source(e, g), targ = target(e, g);
			std::cout << "(" << name[get(vertex_id, src)]
			<< "," << name[get(vertex_id, targ)] << ") ";
		}
		std::cout << std::endl;
		
		// Write out all adjacent vertices
		std::cout << "\tadjacent vertices: ";
		typename graph_traits<Abstract_Graph>::adjacency_iterator ai, ai_end;
		for (tie(ai,ai_end) = adjacent_vertices(v, g);  ai != ai_end; ++ai)
			std::cout << name[get(vertex_id, *ai)] <<  " ";
		std::cout << std::endl;
	}
	
private:
	Abstract_Graph& g;
	const char *name;
};

/*
 * The Graph class
 */
class Abstract_Graph 
{
public:
	Abstract_Graph();
	
private:
	typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS> Graph;
	Graph m_g;
};