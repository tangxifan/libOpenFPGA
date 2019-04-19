#include "rr_graph_error.h"
#include "rr_graph.h"

int t_rr_graph::seg_index_of_cblock(t_rr_type from_rr_type, int to_node) {

  /* Returns the segment number (distance along the channel) of the connection *
   * box from from_rr_type (CHANX or CHANY) to to_node (IPIN).                 */

  if (from_rr_type == CHANX) {
    return (this->rr_nodes_[to_node].xlow());
  } else {
    /* CHANY */
    return (this->rr_nodes_[to_node].ylow());
  }
}

int t_rr_graph::seg_index_of_sblock(int from_node, int to_node) {

  /* Returns the segment number (distance along the channel) of the switch box *
   * box from from_node (CHANX or CHANY) to to_node (CHANX or CHANY).  The     *
   * switch box on the left side of a CHANX segment at (i,j) has seg_index =   *
   * i-1, while the switch box on the right side of that segment has seg_index *
   * = i.  CHANY stuff works similarly.  Hence the range of values returned is *
   * 0 to device_ctx.grid.width()-1 (if from_node is a CHANX) or 0 to device_ctx.grid.height()-1 (if from_node is a CHANY).   */

  t_rr_type from_rr_type, to_rr_type;

  from_rr_type = this->rr_nodes_[from_node].type();
  to_rr_type = this->rr_nodes_[to_node].type();

  if (from_rr_type == CHANX) {
    if (to_rr_type == CHANY) {
      return (this->rr_nodes_[to_node].xlow());
    } else if (to_rr_type == CHANX) {
      if (this->rr_nodes_[to_node].xlow() > this->rr_nodes_[from_node].xlow()) { /* Going right */
        return (this->rr_nodes_[from_node].xhigh());
      } else { /* Going left */
        return (this->rr_nodes_[to_node].xhigh());
      }
    } else {
      rr_graph_throw(__FILE__, __LINE__,
                    "in seg_index_of_sblock: to_node %d is of type %d.\n",
                    to_node, to_rr_type);
      return OPEN; //Should not reach here once thrown
    }
  }
  /* End from_rr_type is CHANX */
  else if (from_rr_type == CHANY) {
    if (to_rr_type == CHANX) {
      return (this->rr_nodes_[to_node].ylow());
    } else if (to_rr_type == CHANY) {
      if (this->rr_nodes_[to_node].ylow() > this->rr_nodes_[from_node].ylow()) { /* Going up */
        return (this->rr_nodes_[from_node].yhigh());
      } else { /* Going down */
        return (this->rr_nodes_[to_node].yhigh());
      }
    } else {
      rr_graph_throw(__FILE__, __LINE__,
         "in seg_index_of_sblock: to_node %d is of type %d.\n",
          to_node, to_rr_type);
      return OPEN; //Should not reach here once thrown
    }
  }
  /* End from_rr_type is CHANY */
  else {
    rr_graph_throw(__FILE__, __LINE__,
      "in seg_index_of_sblock: from_node %d is of type %d.\n",
        from_node, from_rr_type);
    return OPEN; //Should not reach here once thrown
  }
}


