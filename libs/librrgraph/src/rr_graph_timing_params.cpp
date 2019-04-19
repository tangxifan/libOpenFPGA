#include <cstdio>
#include <algorithm>

#include "vtr_memory.h"

#include "rr_graph.h"

/****************** Subroutine definitions *********************************/
/*
 * Returns the index to a t_rr_rc_data matching the specified values.
 *
 * If an existing t_rr_rc_data matches the specified R/C it's index
 * is returned, otherwise the t_rr_rc_data is created.
 *
 * The returned indicies index into DeviceContext.rr_rc_data.
 */
short t_rr_graph::find_create_rr_rc_data(const float R, const float C) {

    auto match = [&](const t_rr_rc_data& val) {
        return val.R == R
            && val.C == C;
    };

    //Just a linear search for now
    auto itr = std::find_if(this->rr_rc_data_.begin(),
                            this->rr_rc_data_.end(),
                            match);

    if (itr == this->rr_rc_data_.end()) {
        //Note found -> create it
        this->rr_rc_data_.emplace_back(R, C);

        itr = --this->rr_rc_data_.end(); //Iterator to inserted value
    }

    return std::distance(this->rr_rc_data_.begin(), itr);
}

void t_rr_graph::add_C_from_switches(int max_len, float C_ipin_cblock) {

  /* This routine finishes loading the C elements of the rr_graph. It assumes *
   * that when you call it the CHANX and CHANY nodes have had their C set to  *
   * their metal capacitance, and everything else has C set to 0.  The graph  *
   * connectivity (edges, switch types etc.) must all be loaded too.  This    *
   * routine will add in the capacitance on the CHANX and CHANY nodes due to: *
   *                                                                          *
   * 1) The output capacitance of the switches coming from OPINs;             *
   * 2) The input and output capacitance of the switches between the various  *
   *    wiring (CHANX and CHANY) segments; and                                *
   * 3) The input capacitance of the input connection block (or buffers       *
   *    separating tracks from the input connection block, if enabled by      *
   *    INCLUDE_TRACK_BUFFERS)                                          */

  int iedge, switch_index, maxlen;
  size_t to_node;
  int icblock, isblock, iseg_low, iseg_high;
  float Cin, Cout;
  t_rr_type from_rr_type, to_rr_type;
  bool * cblock_counted; /* [0..maxlen-1] -- 0th element unused. */
  float *buffer_Cin; /* [0..maxlen-1] */
  bool buffered;
  float *Couts_to_add; /* UDSD */

  cblock_counted = (bool *) vtr::calloc(maxlen, sizeof(bool));
  buffer_Cin = (float *) vtr::calloc(maxlen, sizeof(float));

  std::vector<float> rr_node_C(this->rr_nodes_.size(), 0.); //Stores the final C

  for (size_t inode = 0; inode < this->rr_nodes_.size(); inode++) {

    //The C may have already been partly initialized (e.g. with metal capacitance)
    rr_node_C[inode] += this->rr_nodes_[inode].C();

    from_rr_type = this->rr_nodes_[inode].type();

    if (from_rr_type == CHANX || from_rr_type == CHANY) {

      for (iedge = 0; iedge < this->rr_nodes_[inode].num_edges(); iedge++) {

        to_node = this->rr_nodes_[inode].edge_sink_node(iedge);
        to_rr_type = this->rr_nodes_[to_node].type();

        if (to_rr_type == CHANX || to_rr_type == CHANY) {

          switch_index = this->rr_nodes_[inode].edge_switch(iedge);
          Cin = this->rr_switch_inf_[switch_index].Cin;
          Cout = this->rr_switch_inf_[switch_index].Cout;
          buffered = this->rr_switch_inf_[switch_index].buffered();

          /* If both the switch from inode to to_node and the switch from *
           * to_node back to inode use bidirectional switches (i.e. pass  *
           * transistors), there will only be one physical switch for     *
           * both edges.  Hence, I only want to count the capacitance of  *
           * that switch for one of the two edges.  (Note:  if there is   *
           * a pass transistor edge from x to y, I always build the graph *
           * so that there is a corresponding edge using the same switch  *
           * type from y to x.) So, I arbitrarily choose to add in the    *
           * capacitance in that case of a pass transistor only when      *
           * processing the lower inode number.                           *
           * If an edge uses a buffer I always have to add in the output  *
           * capacitance.  I assume that buffers are shared at the same   *
           * (i,j) location, so only one input capacitance needs to be    *
           * added for all the buffered switches at that location.  If    *
           * the buffers at that location have different sizes, I use the *
           * input capacitance of the largest one.                        */

          if (!buffered && inode < to_node) { /* Pass transistor. */
            rr_node_C[inode] += Cin;
            rr_node_C[to_node] += Cout;
          }

          else if (buffered) {
            /* Prevent double counting of capacitance for UDSD */
            if (this->rr_nodes_[to_node].direction() == BI_DIRECTION) {
              /* For multiple-driver architectures the output capacitance can
               * be added now since each edge is actually a driver */
              rr_node_C[to_node] += Cout;
            }
            isblock = t_rr_graph::seg_index_of_sblock(inode, to_node);
            buffer_Cin[isblock] = std::max(buffer_Cin[isblock], Cin);
          }

        }
        /* End edge to CHANX or CHANY node. */
        else if (to_rr_type == IPIN) {

          if (INCLUDE_TRACK_BUFFERS){
            /* Implements sharing of the track to connection box buffer.
               Such a buffer exists at every segment of the wire at which
               at least one logic block input connects. */
            icblock = t_rr_graph::seg_index_of_cblock(from_rr_type, to_node);
            if (cblock_counted[icblock] == false) {
              rr_node_C[inode] += C_ipin_cblock;
              cblock_counted[icblock] = true;
            }
          } else {
            /* No track buffer. Simply add the capacitance onto the wire */
            rr_node_C[inode] += C_ipin_cblock;
          }
        }
      } /* End loop over all edges of a node. */

      /* Reset the cblock_counted and buffer_Cin arrays, and add buf Cin. */

      /* Method below would be faster for very unpopulated segments, but I  *
       * think it would be slower overall for most FPGAs, so commented out. */

      /*   for (iedge=0;iedge<this->rr_nodes_[inode].num_edges();iedge++) {
       * to_node = this->rr_nodes_[inode].edges[iedge];
       * if (this->rr_nodes_[to_node].type() == IPIN) {
       * icblock = seg_index_of_cblock (from_rr_type, to_node);
       * cblock_counted[icblock] = false;
       * }
       * }     */

      if (from_rr_type == CHANX) {
        iseg_low = this->rr_nodes_[inode].xlow();
        iseg_high = this->rr_nodes_[inode].xhigh();
      } else { /* CHANY */
        iseg_low = this->rr_nodes_[inode].ylow();
        iseg_high = this->rr_nodes_[inode].yhigh();
      }

      for (icblock = iseg_low; icblock <= iseg_high; icblock++) {
        cblock_counted[icblock] = false;
      }

      for (isblock = iseg_low - 1; isblock <= iseg_high; isblock++) {
        rr_node_C[inode] += buffer_Cin[isblock]; /* Biggest buf Cin at loc */
        buffer_Cin[isblock] = 0.;
      }

    }
    /* End node is CHANX or CHANY */
    else if (from_rr_type == OPIN) {

      for (iedge = 0; iedge < this->rr_nodes_[inode].num_edges(); iedge++) {
        switch_index = this->rr_nodes_[inode].edge_switch(iedge);
        to_node = this->rr_nodes_[inode].edge_sink_node(iedge);
        to_rr_type = this->rr_nodes_[to_node].type();

        if (to_rr_type != CHANX && to_rr_type != CHANY)
          continue;

        if (this->rr_nodes_[to_node].direction() == BI_DIRECTION) {
          Cout = this->rr_switch_inf_[switch_index].Cout;
          to_node = this->rr_nodes_[inode].edge_sink_node(iedge); /* Will be CHANX or CHANY */
          rr_node_C[to_node] += Cout;
        }
      }
    }
    /* End node is OPIN. */
  } /* End for all nodes. */

  /* Now we need to add any Cout loads for nets that we previously didn't process
   * Current structures only keep switch information from a node to the next node and
   * not the reverse.  Therefore I need to go through all the possible edges to figure
   * out what the Cout's should be */
  Couts_to_add = (float *) vtr::calloc(this->rr_nodes_.size(), sizeof(float));
  for (size_t inode = 0; inode < this->rr_nodes_.size(); inode++) {
    for (iedge = 0; iedge < this->rr_nodes_[inode].num_edges(); iedge++) {
      switch_index = this->rr_nodes_[inode].edge_switch(iedge);
      to_node = this->rr_nodes_[inode].edge_sink_node(iedge);
      to_rr_type = this->rr_nodes_[to_node].type();
      if (to_rr_type == CHANX || to_rr_type == CHANY) {
        if (this->rr_nodes_[to_node].direction() != BI_DIRECTION) {
          /* Cout was not added in these cases */
          Couts_to_add[to_node] = std::max(Couts_to_add[to_node], this->rr_switch_inf_[switch_index].Cout);

        }
      }
    }
  }
  for (size_t inode = 0; inode < this->rr_nodes_.size(); inode++) {
    rr_node_C[inode] += Couts_to_add[inode];
  }

    //Create the final flywieghted t_rr_rc_data
  for (size_t inode = 0; inode < this->rr_nodes_.size(); inode++) {
        this->rr_nodes_[inode].set_rc_index(t_rr_graph::find_create_rr_rc_data(rr_nodes_[inode].R(), rr_node_C[inode]));
    }

  free(Couts_to_add);
  free(cblock_counted);
  free(buffer_Cin);

  return;
}
