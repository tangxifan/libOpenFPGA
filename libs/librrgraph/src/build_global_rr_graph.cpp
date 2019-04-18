#include "rr_graph.h"

void t_rr_graph::update_global_rr_graph_capacity() {
  /* Update rr_nodes capacities if global routing */
  if (GRAPH_GLOBAL == this->type_) {
    for (size_t inode = 0; inode < this->rr_nodes_.size(); inode++) {
      if (this->rr_nodes_[i].type() == CHANX) {
        int ylow = this->rr_nodes_[i].ylow();
        this->rr_nodes_[i].set_capacity(this->chan_width_.x_list[ylow]);
      }
      if (this->rr_nodes_[i].type() == CHANY) {
        int xlow = this->rr_nodes_[i].xlow();
        this->rr_nodes_[i].set_capacity(this->chan_width_.y_list[xlow]);
      }
    }
  }

  return;
}
