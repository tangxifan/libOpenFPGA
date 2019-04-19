#include <algorithm> 
#include <vector>

#include "rr_node.h"
#include "rr_graph_error.h"

/* Member function of "t_rr_node" used to retrieve a routing *
 * resource type string by its index, which is defined by           *
 * "t_rr_type type".												*/

const char *t_rr_node::type_string() const {
	return rr_node_typename[type()];
}

short t_rr_node::xlow() const {
	return xlow_;
}

short t_rr_node::ylow() const {
	return ylow_;
}

short t_rr_node::xhigh() const {
    return xhigh_;
}

short t_rr_node::yhigh() const {
    return yhigh_;
}

short t_rr_node::ptc_num() const {
	return ptc_.pin_num; //TODO eventually remove
}

short t_rr_node::pin_num() const {
    if (type() != IPIN && type() != OPIN) {
        RR_GRAPH_THROW("Attempted to access RR node 'pin_num' for non-IPIN/OPIN type '%s'", type_string());
    }
    return ptc_.pin_num;
}

short t_rr_node::track_num() const {
    if (type() != CHANX && type() != CHANY) {
        RR_GRAPH_THROW("Attempted to access RR node 'track_num' for non-CHANX/CHANY type '%s'", type_string());
    }
    return ptc_.track_num;
}

short t_rr_node::class_num() const {
    if (type() != SOURCE && type() != SINK) {
        RR_GRAPH_THROW("Attempted to access RR node 'class_num' for non-SOURCE/SINK type '%s'", type_string());
    }
    return ptc_.class_num;
}

short t_rr_node::cost_index() const {
	return cost_index_;
}

short t_rr_node::rc_index() const {
	return rc_index_;
}

short t_rr_node::capacity() const {
	return capacity_;
}

short t_rr_node::fan_in() const {
	return fan_in_;
}

e_direction t_rr_node::direction() const {
    if (type() != CHANX && type() != CHANY) {
        RR_GRAPH_THROW("Attempted to access RR node 'direction' for non-channel type '%s'", type_string());
    }
	return dir_side_.direction;
}

const char* t_rr_node::direction_string() const{
	if (direction() == INC_DIRECTION){
		return "INC_DIR";
	} else if (direction() == DEC_DIRECTION){
		return "DEC_DIR";
	} else if (direction() == BI_DIRECTION){
		return "BI_DIR";
    }

	VTR_ASSERT(direction() == NO_DIRECTION);
    return "NO_DIR";
}

e_side t_rr_node::side() const {
    if (type() != IPIN && type() != OPIN) {
        RR_GRAPH_THROW("Attempted to access RR node 'side' for non-IPIN/OPIN type '%s'", type_string());
    }
	return dir_side_.side;
}

const char* t_rr_node::side_string() const {
    return SIDE_STRING[side()];
}

//Returns the max 'length' over the x or y direction
short t_rr_node::length() const {
	return std::max(yhigh_ - ylow_, xhigh_ - xlow_);
}

void t_rr_node::set_type(t_rr_type new_type) {
    type_ = new_type;
}

/*
	Pass in two coordinate variables describing location of node.
	They do not have to be in any particular order.
*/
void t_rr_node::set_coordinates(short x1, short y1, short x2, short y2) {
	if (x1 < x2) {
		xlow_ = x1;
        xhigh_ = x2;
	} else {
		xlow_ = x2;
        xhigh_ = x1;
	}

	if (y1 < y2) {
		ylow_ = y1;
        yhigh_ = y2;
	} else {
		ylow_ = y2;
        yhigh_ = y1;
	}
}

void t_rr_node::set_ptc_num(short new_ptc_num) {
	ptc_.pin_num = new_ptc_num; //TODO: eventually remove
}

void t_rr_node::set_pin_num(short new_pin_num) {
    if (type() != IPIN && type() != OPIN) {
        RR_GRAPH_THROW("Attempted to set RR node 'pin_num' for non-IPIN/OPIN type '%s'", type_string());
    }
    ptc_.pin_num = new_pin_num;
}

void t_rr_node::set_track_num(short new_track_num) {
    if (type() != CHANX && type() != CHANY) {
        RR_GRAPH_THROW("Attempted to set RR node 'track_num' for non-CHANX/CHANY type '%s'", type_string());
    }
    ptc_.track_num = new_track_num;
}

void t_rr_node::set_class_num(short new_class_num) {
    if (type() != SOURCE && type() != SINK) {
        RR_GRAPH_THROW("Attempted to set RR node 'class_num' for non-SOURCE/SINK type '%s'", type_string());
    }
    ptc_.class_num = new_class_num;
}

void t_rr_node::set_cost_index(size_t new_cost_index) {
	cost_index_ = new_cost_index;
}

void t_rr_node::set_rc_index(short new_rc_index) {
	rc_index_ = new_rc_index;
}

void t_rr_node::set_capacity(short new_capacity) {
    VTR_ASSERT(new_capacity >= 0);
	capacity_ = new_capacity;
}

void t_rr_node::set_fan_in(short new_fan_in) {
    VTR_ASSERT(new_fan_in >= 0);
	fan_in_ = new_fan_in;
}

short t_rr_node::add_edge(int sink_node, int iswitch) {
    if (edges_capacity_ == num_edges_) {

        constexpr size_t MAX_EDGE_COUNT = std::numeric_limits<decltype(edges_capacity_)>::max();
        if (edges_capacity_ == MAX_EDGE_COUNT) {
            RR_GRAPH_THROW("Maximum RR Node out-edge count (%zu) exceeded", MAX_EDGE_COUNT);
        }

        //Grow
        size_t new_edges_capacity = std::max<size_t>(1, 2 * edges_capacity_);
        new_edges_capacity = std::min(new_edges_capacity, MAX_EDGE_COUNT); //Clip to maximum count
        auto new_edges = std::make_unique<t_rr_edge[]>(new_edges_capacity);

        //Copy
        std::copy_n(edges_.get(), num_edges_, new_edges.get());

        //Replace
        edges_ = std::move(new_edges);
        edges_capacity_ = new_edges_capacity;
    }

    VTR_ASSERT(num_edges_ < edges_capacity_);

    edges_[num_edges_].sink_node = sink_node;
    edges_[num_edges_].switch_id = iswitch;

    ++num_edges_;

    return num_edges_;
}

void t_rr_node::shrink_to_fit() {
    //Shrink
    auto new_edges = std::make_unique<t_rr_edge[]>(num_edges_);

    //Copy
    std::copy_n(edges_.get(), num_edges_, new_edges.get());

    //Replace
    edges_ = std::move(new_edges);
    edges_capacity_ = num_edges_;
}


void t_rr_node::set_num_edges(short new_num_edges) {
    VTR_ASSERT(new_num_edges >= 0);
    num_edges_ = new_num_edges;
    edges_capacity_ = new_num_edges;

    edges_ = std::make_unique<t_rr_edge[]>(num_edges_);
}

/* A mutator function to set the number of non configurable edges for a rr_node */
void t_rr_node::set_num_non_configurable_edges(short new_num_non_conf_edges) {
  num_non_configurable_edges_ = new_num_non_conf_edges;

  return;
}


void t_rr_node::set_direction(e_direction new_direction) {
    if (type() != CHANX && type() != CHANY) {
        RR_GRAPH_THROW("Attempted to set RR node 'direction' for non-channel type '%s'", type_string());
    }
	dir_side_.direction = new_direction;
}

void t_rr_node::set_side(e_side new_side) {
    if (type() != IPIN && type() != OPIN) {
        RR_GRAPH_THROW("Attempted to set RR node 'side' for non-channel type '%s'", type_string());
    }
	dir_side_.side = new_side;
}

void t_rr_node::set_edge_sink_node(short iedge, int sink_node) {
    VTR_ASSERT(iedge < num_edges());
    VTR_ASSERT(sink_node >= 0);
    edges_[iedge].sink_node = sink_node;
}

void t_rr_node::set_edge_switch(short iedge, short switch_index, std::vector<t_rr_switch_inf> switch_list) {
    VTR_ASSERT(iedge < num_edges());
    VTR_ASSERT(switch_index >= 0);
    edges_[iedge].switch_id = switch_index;
    /* Assign the pointer */
    edges_[iedge].switch_inf_ptr = switch_list.data() + switch_index;
}

/* This function aims to identify an edge of a rr_node is configurable
 * Previous this function is a member of t_rr_node 
 * However, it really depends on a data structure rr_switch_inf in rr_graph
 * Therefore, it is more suitable to change it to a member function of rr_graph  
 */
bool t_rr_node::edge_is_configurable(short iedge) const {
  return edges_[iedge].switch_inf_ptr->configurable();
}

/* This function aims to get the resistance of a rr_node 
 * Previous this function is a member of t_rr_node 
 * However, it really depends on a data structure rr_rc_data in rr_graph
 * Therefore, it is more suitable to change it to a member function of rr_graph  
 */
float t_rr_node::R() const {
  return rc_data_ptr->R;
}

/* This function aims to get the capacitance of a rr_node 
 * Previous this function is a member of t_rr_node 
 * However, it really depends on a data structure rr_rc_data in rr_graph
 * Therefore, it is more suitable to change it to a member function of rr_graph  
 */
float t_rr_node::C() const {
  return rc_data_ptr->C;
}

/* This function aims to check if the internal assumption of a rr_node is reasonable
 * Previous this function is a member of t_rr_node 
 * However, it really depends on a data structure rr_rc_data in rr_graph
 * Therefore, it is more suitable to change it to a member function of rr_graph  
 */
bool t_rr_node::validate() const {
  //Check internal assumptions about RR node are valid

  if (num_edges_ > edges_capacity_) {
    RR_GRAPH_THROW("RR Node number of edges exceeded edge capacity");
  }

  short iedge = 0;
  for (auto edge : edges()) {
    if (edge < num_configurable_edges()) {
      if (false == edge_is_configurable(edge)) {
        RR_GRAPH_THROW("RR Node non-configurable edge found in configurable edge list");
      }
    } else {
      if (true == edge_is_configurable(edge)) {
        RR_GRAPH_THROW("RR Node configurable edge found in non-configurable edge list");
      }
    }
    ++iedge;
  }

  if (iedge != num_edges()) {
    RR_GRAPH_THROW("RR Node Edge iteration does not match edge size");
  }

  return true;
}

/* This function aims to partition edges of a rr_node in the graph 
 * Partitions all edges so that configurable and non-configurable edges
 * are organized for efficient access.
 * 
 * Must be called before configurable_edges(), non_configurable_edges(),
 * num_configurable_edges(), num_non_configurable_edges() to ensure they
 * are correct.
 *
 * Previous this function is a member of t_rr_node 
 * However, it really depends on a data structure rr_rc_data in rr_graph
 * Therefore, it is more suitable to change it to a member function of rr_graph  
 */
void t_rr_node::partition_edges() {
  auto is_configurable = [&](const t_rr_edge& edge) {
    return edge.switch_inf_ptr->configurable();
  };

  //Partition the edges so the first set of edges are all configurable, and the later are not
  auto first_non_config_edge = std::partition(edges_.get(), 
                                              edges_.get() + num_edges_, 
                                              is_configurable);

  size_t num_conf_edges = std::distance(edges_.get(), first_non_config_edge);
  size_t num_non_conf_edges = num_edges_ - num_conf_edges; //Note we calculate using the size_t to get full range

  //Check that within allowable range (no overflow when stored as num_non_configurable_edges_
  if (num_non_conf_edges > std::numeric_limits<decltype(num_non_configurable_edges_)>::max()) {
    RR_GRAPH_THROW("Exceeded RR node maximum number of non-configurable edges");
  }
  num_non_configurable_edges_ = num_non_conf_edges; //Narrowing
}

t_rr_rc_data::t_rr_rc_data(float Rval, float Cval)
    : R(Rval)
    , C(Cval)
    {}

