#include "rr_graph.h"

void t_rr_graph::update_global_rr_graph_capacity() {
  /* Update rr_nodes capacities if global routing */
  if (GRAPH_GLOBAL == this->type_) {
    for (size_t inode = 0; inode < this->rr_nodes_.size(); inode++) {
      if (this->rr_nodes_[inode].type() == CHANX) {
        int ylow = this->rr_nodes_[inode].ylow();
        this->rr_nodes_[inode].set_capacity(this->chan_width_.get_x_list_member(ylow));
      }
      if (this->rr_nodes_[inode].type() == CHANY) {
        int xlow = this->rr_nodes_[inode].xlow();
        this->rr_nodes_[inode].set_capacity(this->chan_width_.get_y_list_member(xlow));
      }
    }
  }

  return;
}
