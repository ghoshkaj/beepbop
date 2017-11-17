// For the location index. There are different types of indexes available.
// This will work for all input files keeping the index in memory.
#include <osmium/index/map/flex_mem.hpp>

// For the osmium::geom::haversine::distance() function
#include <osmium/geom/haversine.hpp>

// For the NodeLocationForWays handler
#include <osmium/handler/node_locations_for_ways.hpp>

// The type of index used. This must match the include file above
using index_type = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>;

// The location handler always depends on the index type
using location_handler_type = osmium::handler::NodeLocationsForWays<index_type>;

#include <osmium/handler.hpp>
// For osmium::apply()
#include <osmium/visitor.hpp>
#include <protozero/exception.hpp>
#include <iostream> // for std::cout, std::cerr
// Allow any format of input files (XML, PBF, ...)
#include <osmium/io/any_input.hpp>

// GBL structures and methods
#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>

#include <fstream>

using namespace boost;
typedef adjacency_list < listS, vecS, directedS, no_property, property < edge_weight_t, int > > graph_t;
typedef graph_traits < graph_t >::vertex_descriptor vertex_descriptor;
typedef std::pair<osmium::object_id_type, osmium::object_id_type> Edge;
graph_t g;
std::vector<Edge> osm_edges;
std::vector<int> osm_weights;
std::map<osmium::object_id_type, vertex_descriptor> osmId_to_index;
std::map<vertex_descriptor, osmium::object_id_type> index_to_osmId;

struct WayHandler : public osmium::handler::Handler {

void node(const osmium::Node& node) {
	auto v = add_vertex(g);
	osmId_to_index[node.positive_id()] = v;
	index_to_osmId[v] = node.positive_id();
}

void way(const osmium::Way& way) {
auto prev =  way.nodes().begin();
auto current = way.nodes().begin()+1;    
    for (; current != way.nodes().end(); prev++,current++) {
            // std::cout << prev->positive_ref() << ", " << current->positive_ref() << ", " << osmium::geom::haversine::distance(prev->location(), current->location()) << '\n';
           osm_edges.push_back(Edge(osmId_to_index[prev->positive_ref()], osmId_to_index[current->positive_ref()]));
           osm_weights.push_back(osmium::geom::haversine::distance(prev->location(), current->location()));
        }
    }
};

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " OSMFILE\n";
        std::exit(1);
    }

    // Initialize the reader with the filename from the command line and
    // tell it to only read nodes and ways.
    osmium::io::Reader reader{argv[1], osmium::osm_entity_bits::node | osmium::osm_entity_bits::way};

    // The index to hold node locations.
    index_type index;

    // The location handler will add the node locations to the index and then
    // to the ways
    location_handler_type location_handler{index};

    WayHandler way_handler;
    osmium::apply(reader, location_handler, way_handler); 
    // Output the length. The haversine function calculates it in meters,
    // so we first devide by 1000 to get kilometers.

    // std::cout << osm_edges.size() << std::endl;
    // std::cout << osm_weights.size() << std::endl;

    const int num_nodes = osm_edges.size() == osm_weights.size() ? osm_weights.size(): 0;
    // if (num_nodes == 0) exit;

    Edge edge_array[num_nodes]; std::copy(osm_edges.begin(), osm_edges.end(), edge_array);
    int weights[num_nodes]; std::copy(osm_weights.begin(), osm_weights.end(), weights);

    int num_arcs = sizeof(edge_array) / sizeof(Edge);
    graph_t g(edge_array, edge_array + num_arcs, weights, num_nodes);
    property_map<graph_t, edge_weight_t>::type weightmap = get(edge_weight, g);
    std::vector<vertex_descriptor> p(num_vertices(g));
    std::vector<int> d(num_vertices(g));
    vertex_descriptor s = vertex(osm_edges[0].first, g);

    // std::cout << "edges(g) = ";
    //    graph_traits<graph_t>::edge_iterator ei, ei_end;
    //    for (std::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
    //        std::cout << "(" << index_to_osmId[source(*ei, g)] 
    //                  << "," << index_to_osmId[target(*ei, g)] << ") ";
    //    std::cout << std::endl;

    dijkstra_shortest_paths(g, s, predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, g))).
    						                       distance_map(boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, g))));

    // std::cout << "distances and parents:" << std::endl;
    graph_traits < graph_t >::vertex_iterator vi, vend;
    for (boost::tie(vi, vend) = vertices(g); vi != vend; ++vi) {
    	// std::cout << index_to_osmId[*vi] << std::endl;
    	// std::cout << "distance(" << index_to_osmId[*vi] << ") = " << d[*vi] << ", ";
    	// std::cout << "parent(" << index_to_osmId[*vi] << ") = " << index_to_osmId[p[*vi]] << std::endl;
    }
    std::cout << std::endl;

    typedef std::vector<vertex_descriptor> path_type;

    
    path_type path;
    // auto const end = index_to_osmId[49737885];
    auto const end = osmId_to_index[49737885];
    auto v = end;
    for (vertex_descriptor
           u = p[v];     // Start by setting 'u' to the
                         // destintaion node's predecessor
       u != v;           // Keep tracking the path until we get to the source
       v = u, u = p[v])  // Set the current vertex to the current predecessor,
                         // and the predecessor to one level up
    {

        path.push_back(u);
    }

    std::cout << "{ \"type\": \"LineString\", \"properties\": {}, \"coordinates\": [";
    for (auto i = path.begin(); i != path.end(); i++) {
        // std::cout << "https://www.openstreetmap.org/node/" << index_to_osmId[u] << std::endl;
        auto id = index_to_osmId[*i];
        auto location = index.get(id);
        std::cout << "["<<location.lon() << "," << location.lat();
        if ( path.end() - i == 1 ) { std::cout << "]"; }
        else { std::cout << "],"; };
    }
    std::cout << "] }" << std::endl;


}
 
