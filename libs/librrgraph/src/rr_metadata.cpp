#include "rr_graph.h"

const t_metadata_value* t_rr_graph::find_rr_node_metadata(int src_node, std::string key) {

    if ( (0 == this->rr_node_metadata_.size())
      || (0 == this->rr_node_metadata_.count(src_node)) ) {
        return nullptr;
    }
    auto& data = this->rr_node_metadata_.at(src_node);
    return data.one(key);
}

void t_rr_graph::add_rr_node_metadata(int src_node, std::string key, std::string value) {

  if (0 == this->rr_node_metadata_.count(src_node)) {
    this->rr_node_metadata_.emplace(src_node, t_metadata_dict());
  }
  auto& data = this->rr_node_metadata_.at(src_node);
  data.add(key, value);

  return;
}

const t_metadata_value* t_rr_graph::find_rr_edge_metadata(int src_node, int sink_id, short switch_id, std::string key) {
    auto rr_edge = std::make_tuple(src_node, sink_id, switch_id);

    auto iter = this->rr_edge_metadata_.find(rr_edge);
    if (iter == this->rr_edge_metadata_.end()) {
        return nullptr;
    }

    return iter->second.one(key);
}

void t_rr_graph::add_rr_edge_metadata(int src_node, int sink_id, short switch_id, 
                                      std::string key, std::string value) {
  auto rr_edge = std::make_tuple(src_node, sink_id, switch_id);
  if (0 == this->rr_edge_metadata_.count(rr_edge)) {
    this->rr_edge_metadata_.emplace(rr_edge, t_metadata_dict());
  }
  auto& data = this->rr_edge_metadata_.at(rr_edge);
  data.add(key, value);
}

