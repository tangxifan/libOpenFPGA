#include "vpr_error.h"

#include "rr_graph.h"

/* Member functions of "t_rr_graph" */


/* Critical function to fetch a wanted rr_node from rr_graph 
 * This is one of the most used functions
 * User should provide:
 * 1. Coordinator of rr_node
 * 2. Type of rr_node
 * 3. ID of rr_node 
 * 4. side of rr_node if this is OPIN/IPIN 
 */

/* Partition the rr graph edges for efficient access to configurable/non-configurable
 * edge subsets. Must be done after RR switches have been allocated
 */
void t_rr_graph::partition_rr_graph_edges() {
  for (size_t inode = 0; inode < this->rr_nodes_.size(); ++inode) {
    this->rr_nodes_[inode].partition_edges();

    VTR_ASSERT_SAFE(this->rr_nodes_[inode].validate());
  }

  return;
}

int t_rr_graph::get_max_chan_width() {
    int max_chan_width = (rr_graph_type ? 1 : chan_width.max());
    VTR_ASSERT(max_chan_width > 0);

    return max_chan_width;
}

