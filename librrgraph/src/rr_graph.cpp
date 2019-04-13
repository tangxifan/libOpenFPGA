#include "rr_graph.h"
#include "vpr_error.h"

/* Member functions of "t_rr_graph" */

/* Constructor for routing channels */
void init_chan(const DeviceGrid& grid, int cfactor, t_chan_width_dist chan_width_dist) {

    /* Assigns widths to channels (in tracks).  Minimum one track          *
     * per channel. The channel distributions read from the architecture  *
     * file are scaled by cfactor.                                         */
    t_chan chan_x_dist = chan_width_dist.chan_x_dist;
    t_chan chan_y_dist = chan_width_dist.chan_y_dist;

    chan_width.x_list.resize(grid.height());
    chan_width.y_list.resize(grid.width());

    if (grid.height() > 1) {
        int num_channels = grid.height() - 1;
        VTR_ASSERT(num_channels > 0);
        float separation = 1.0 / num_channels; /* Norm. distance between two channels. */

        for (size_t i = 0; i < grid.height(); ++i) {
            float y = float(i) / num_channels;
            chan_width.x_list[i] = (int) floor(cfactor * comp_width(&chan_x_dist, y, separation) + 0.5);
            chan_width.x_list[i] = max(chan_width.x_list[i], 1); //Minimum channel width 1
        }
    }

    if (grid.width() > 1) {
        int num_channels = grid.width() - 1;
        VTR_ASSERT(num_channels > 0);
        float separation = 1.0 / num_channels; /* Norm. distance between two channels. */

        for (size_t i = 0; i < grid.width(); ++i) { //-2 for no perim channels
            float x = float(i) / num_channels;

            chan_width.y_list[i] = (int) floor(cfactor * comp_width(&chan_y_dist, x, separation) + 0.5);
            chan_width.y_list[i] = max(chan_width.y_list[i], 1); //Minimum channel width 1
        }
    }

    chan_width.max = 0;
    chan_width.x_max = chan_width.y_max = INT_MIN;
    chan_width.x_min = chan_width.y_min = INT_MAX;
    for (size_t i = 0; i < grid.height(); ++i) {
        chan_width.max = max(chan_width.max, chan_width.x_list[i]);
        chan_width.x_max = max(chan_width.x_max, chan_width.x_list[i]);
        chan_width.x_min = min(chan_width.x_min, chan_width.x_list[i]);
    }
    for (size_t i = 0; i < grid.width(); ++i) {
        chan_width.max = max(chan_width.max, chan_width.y_list[i]);
        chan_width.y_max = max(chan_width.y_max, chan_width.y_list[i]);
        chan_width.y_min = min(chan_width.y_min, chan_width.y_list[i]);
    }

#ifdef VERBOSE
    VTR_LOG("\n");
    VTR_LOG("device_ctx.chan_width.x_list:\n");
    for (size_t i = 0; i < grid.height(); ++i) {
        VTR_LOG("%d  ", chan_width.x_list[i]);
    }
    VTR_LOG("\n");
    VTR_LOG("device_ctx.chan_width.y_list:\n");
    for (size_t i = 0; i < grid.width(); ++i) {
        VTR_LOG("%d  ", chan_width.y_list[i]);
    }
    VTR_LOG("\n");
#endif

    return chan_width;
}


/* Critical function to fetch a wanted rr_node from rr_graph 
 * This is one of the most used functions
 * User should provide:
 * 1. Coordinator of rr_node
 * 2. Type of rr_node
 * 3. ID of rr_node 
 * 4. side of rr_node if this is OPIN/IPIN 
 */
int t_rr_graph::get_rr_node_index(int x, int y, 
                                  t_rr_type rr_type, 
                                  int ptc, e_side side) {
  /*
   * Returns the index of the specified routing resource node.  (x,y) are
   * the location within the FPGA, rr_type specifies the type of resource,
   * and ptc gives the number of this resource.  ptc is the class number,
   * pin number or track number, depending on what type of resource this
   * is.  All ptcs start at 0 and go up to pins_per_clb-1 or the equivalent.
   * There are type->num_class SOURCEs + SINKs, type->num_pins IPINs + OPINs,
   * and max_chan_width CHANX and CHANY (each).
   *
   * Note that for segments (CHANX and CHANY) of length > 1, the segment is
   * given an rr_index based on the (x,y) location at which it starts (i.e.
   * lowest (x,y) location at which this segment exists).
   * This routine also performs error checking to make sure the node in
   * question exists.
   *
   * The 'side' argument only applies to IPIN/OPIN types, and specifies which
   * side of the grid tile the node should be located on. The value is ignored
   * for non-IPIN/OPIN types
   */
  if (rr_type == IPIN || rr_type == OPIN) {
      VTR_ASSERT_MSG(side != NUM_SIDES, "IPIN/OPIN must specify desired side (can not be default NUM_SIDES)");
  } else {
      VTR_ASSERT(rr_type != IPIN && rr_type != OPIN);
      side = SIDES[0];
  }

  /*
  int iclass;
   */

  VTR_ASSERT(ptc >= 0);
  /* FIXME: Find a smarter way to check the range of x and y without using grid 
  VTR_ASSERT(x >= 0 && x < int(device_ctx.grid.width()));
  VTR_ASSERT(y >= 0 && y < int(device_ctx.grid.height()));

  t_type_ptr type = device_ctx.grid[x][y].type;
  */

  /* Currently need to swap x and y for CHANX because of chan, seg convention */
  if (CHANX == rr_type) {
      std::swap(x, y);
  }

  /* Start of that block.  */
  const std::vector<int>& lookup = rr_node_indices[rr_type][x][y][side];

  /* Check valid ptc num */
  VTR_ASSERT(ptc >= 0);

  /* Check if the rr_type matches the pin type of a grid */
  /*
  switch (rr_type) {
      case SOURCE:
          VTR_ASSERT(ptc < type->num_class);
          VTR_ASSERT(type->class_inf[ptc].type == DRIVER);
          break;
      case SINK:
          VTR_ASSERT(ptc < type->num_class);
          VTR_ASSERT(type->class_inf[ptc].type == RECEIVER);
          break;
      case OPIN:
          VTR_ASSERT(ptc < type->num_pins);
          iclass = type->pin_class[ptc];
          VTR_ASSERT(type->class_inf[iclass].type == DRIVER);
          break;
      case IPIN:
          VTR_ASSERT(ptc < type->num_pins);
          iclass = type->pin_class[ptc];
          VTR_ASSERT(type->class_inf[iclass].type == RECEIVER);
          break;
      case CHANX:
      case CHANY:
          break;
      default:
          vpr_throw(VPR_ERROR_ROUTE, __FILE__, __LINE__,
                  "Bad rr_node passed to get_rr_node_index.\n"
                  "Request for type=%d ptc=%d at (%d, %d).\n",
                  rr_type, ptc, x, y);
  }
  */

  return ((unsigned)ptc < lookup.size() ? lookup[ptc] : -1);
}

/* Partition the rr graph edges for efficient access to configurable/non-configurable
 * edge subsets. Must be done after RR switches have been allocated
 */
void t_rr_graph::partition_rr_graph_edges() {
  for (size_t inode = 0; inode < rr_nodes.size(); ++inode) {
    rr_nodes[inode].partition_edges();

    VTR_ASSERT_SAFE(rr_nodes[inode].validate());
  }

  return;
}

/* Check if the rr_graph is correct after allocation */
void t_rr_graph::check_rr_graph(const t_graph_type graph_type,
                                const DeviceGrid& grid,
                                const t_type_ptr types) {

    e_route_type route_type = DETAILED;
    if (graph_type == GRAPH_GLOBAL) {
        route_type = GLOBAL;
    }

    auto total_edges_to_node = std::vector<int>(rr_nodes.size());
    auto switch_types_from_current_to_node = std::vector<unsigned char>(rr_nodes.size());
    const int num_rr_switches = rr_switch_inf.size();

    for (size_t inode = 0; inode < rr_nodes.size(); inode++) {

        rr_nodes[inode].validate();

        /* Ignore any uninitialized rr_graph nodes */
        if ((rr_nodes[inode].type() == SOURCE)
                && (rr_nodes[inode].xlow() == 0) && (rr_nodes[inode].ylow() == 0)
                && (rr_nodes[inode].xhigh() == 0) && (rr_nodes[inode].yhigh() == 0)) {
            continue;
        }

        t_rr_type rr_type = rr_nodes[inode].type();
        int num_edges = rr_nodes[inode].num_edges();

        check_rr_node(inode, route_type, device_ctx);

        /* Check all the connectivity (edges, etc.) information.                    */

        std::map<int,std::vector<int>> edges_from_current_to_node;
        for (int iedge = 0; iedge < num_edges; iedge++) {
            int to_node = rr_nodes[inode].edge_sink_node(iedge);

            check_rr_edge(inode, iedge, to_node);

            if (to_node < 0 || to_node >= (int) rr_nodes.size()) {
                vpr_throw(VPR_ERROR_ROUTE, __FILE__, __LINE__,
                        "in check_rr_graph: node %d has an edge %d.\n"
                        "\tEdge is out of range.\n", inode, to_node);
            }

            edges_from_current_to_node[to_node].push_back(iedge);
            total_edges_to_node[to_node]++;

            auto switch_type = rr_nodes[inode].edge_switch(iedge);

            if (switch_type < 0 || switch_type >= num_rr_switches) {
                vpr_throw(VPR_ERROR_ROUTE, __FILE__, __LINE__,
                        "in check_rr_graph: node %d has a switch type %d.\n"
                        "\tSwitch type is out of range.\n",
                        inode, switch_type);
            }
        } /* End for all edges of node. */

        //Check that multiple edges between the same from/to nodes make sense
        for (int iedge = 0; iedge < num_edges; iedge++) {
            int to_node = rr_nodes[inode].edge_sink_node(iedge);

            if (edges_from_current_to_node[to_node].size() == 1) continue; //Single edges are always OK

            VTR_ASSERT_MSG(edges_from_current_to_node[to_node].size() > 1, "Expect multiple edges");

            t_rr_type to_rr_type = rr_nodes[to_node].type();

            //Only expect chan <-> chan connections to have multiple edges
            if ((to_rr_type != CHANX && to_rr_type != CHANY)
                || (rr_type != CHANX && rr_type != CHANY)) {
                vpr_throw(VPR_ERROR_ROUTE, __FILE__, __LINE__,
                        "in check_rr_graph: node %d (%s) connects to node %d (%s) %zu times - multi-connections only expected for CHAN->CHAN.\n",
                        inode, rr_node_typename[rr_type], to_node, rr_node_typename[to_rr_type], edges_from_current_to_node[to_node].size());
            }

            //Between two wire segments
            VTR_ASSERT_MSG(to_rr_type == CHANX || to_rr_type == CHANY, "Expect channel type");
            VTR_ASSERT_MSG(rr_type == CHANX || rr_type == CHANY, "Expect channel type");

            //While multiple connections between the same wires can be electrically legal,
            //they are redundant if they are of the same switch type.
            //
            //Identify any such edges with identical switches
            std::map<short,int> switch_counts;
            for (auto edge : edges_from_current_to_node[to_node]) {
                auto edge_switch = rr_nodes[inode].edge_switch(edge);

                switch_counts[edge_switch]++;
            }

            //Tell the user about any redundant edges
            for (auto kv : switch_counts) {
                if (kv.second <= 1) continue;

                auto switch_type = rr_switch_inf[kv.first].type();

                VPR_THROW(VPR_ERROR_ROUTE, "in check_rr_graph: node %d has %d redundant connections to node %d of switch type %d (%s)", 
                          inode, kv.second, to_node, kv.first, SWITCH_TYPE_STRINGS[size_t(switch_type)]);
            }
        }

        /* Slow test could leave commented out most of the time. */
        check_unbuffered_edges(inode);

        //Check that all config/non-config edges are appropriately organized
        for (auto edge : rr_nodes[inode].configurable_edges()) {
            if (!rr_nodes[inode].edge_is_configurable(edge)) {
                VPR_THROW(VPR_ERROR_ROUTE, "in check_rr_graph: node %d edge %d is non-configurable, but in configurable edges",
                        inode, edge);
            }
        }

        for (auto edge : rr_nodes[inode].non_configurable_edges()) {
            if (rr_nodes[inode].edge_is_configurable(edge)) {
                VPR_THROW(VPR_ERROR_ROUTE, "in check_rr_graph: node %d edge %d is configurable, but in non-configurable edges",
                        inode, edge);
            }
        }

    } /* End for all rr_nodes */

    /* I built a list of how many edges went to everything in the code above -- *
     * now I check that everything is reachable.                                */
    bool is_fringe_warning_sent = false;

    for (size_t inode = 0; inode < rr_nodes.size(); inode++) {
        t_rr_type rr_type = rr_nodes[inode].type();

        if (rr_type != SOURCE) {
            if (total_edges_to_node[inode] < 1 && !rr_node_is_global_clb_ipin(inode)) {

                /* A global CLB input pin will not have any edges, and neither will  *
                 * a SOURCE or the start of a carry-chain.  Anything else is an error.
                 * For simplicity, carry-chain input pin are entirely ignored in this test
                 */
                bool is_chain = false;
                if (rr_type == IPIN) {
                    t_type_ptr type = grid[device_ctx.rr_nodes[inode].xlow()][rr_nodes[inode].ylow()].type;
                    for (const t_fc_specification& fc_spec : types[type->index].fc_specs) {
                        if (fc_spec.fc_value == 0 && fc_spec.seg_index == 0) {
                            is_chain = true;
                        }
                    }
                }

                const auto& node = rr_nodes[inode];


                bool is_fringe = ((rr_nodes[inode].xlow() == 1)
                        || (rr_nodes[inode].ylow() == 1)
                        || (rr_nodes[inode].xhigh() == int(grid.width()) - 2)
                        || (rr_nodes[inode].yhigh() == int(grid.height()) - 2));
                bool is_wire = (rr_nodes[inode].type() == CHANX
                        || rr_nodes[inode].type() == CHANY);

                if (!is_chain && !is_fringe && !is_wire) {
                    if (node.type() == IPIN || node.type() == OPIN) {

                        if (has_adjacent_channel(node, grid)) {
                            auto block_type = grid[node.xlow()][node.ylow()].type;
                            VTR_LOG_ERROR(
                                    "in check_rr_graph: node %d (%s) at (%d,%d) block=%s side=%s has no fanin.\n",
                                    inode, node.type_string(), node.xlow(), node.ylow(), block_type->name, node.side_string());
                        }
                    } else {
                        VTR_LOG_ERROR( "in check_rr_graph: node %d (%s) has no fanin.\n",
                                inode, device_ctx.rr_nodes[inode].type_string());
                    }
                } else if (!is_chain && !is_fringe_warning_sent) {
                    VTR_LOG_WARN(
                            "in check_rr_graph: fringe node %d %s at (%d,%d) has no fanin.\n"
                            "\t This is possible on a fringe node based on low Fc_out, N, and certain lengths.\n",
                            inode, rr_nodes[inode].type_string(), rr_nodes[inode].xlow(), rr_nodes[inode].ylow());
                    is_fringe_warning_sent = true;
                }
            }
        } else { /* SOURCE.  No fanin for now; change if feedthroughs allowed. */
            if (total_edges_to_node[inode] != 0) {
                VTR_LOG_ERROR(
                        "in check_rr_graph: SOURCE node %d has a fanin of %d, expected 0.\n",
                        inode, total_edges_to_node[inode]);
            }
        }
    }
}

void t_rr_graph::add_rr_graph_C_from_switches(const DeviceGrid& grid, float C_ipin_cblock) {

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
	 *    INCLUDE_TRACK_BUFFERS)                                    	    */

	int iedge, switch_index, maxlen;
    size_t to_node;
	int icblock, isblock, iseg_low, iseg_high;
	float Cin, Cout;
	t_rr_type from_rr_type, to_rr_type;
	bool * cblock_counted; /* [0..maxlen-1] -- 0th element unused. */
	float *buffer_Cin; /* [0..maxlen-1] */
	bool buffered;
	float *Couts_to_add; /* UDSD */

	maxlen = max(grid.width(), grid.height());
	cblock_counted = (bool *) vtr::calloc(maxlen, sizeof(bool));
	buffer_Cin = (float *) vtr::calloc(maxlen, sizeof(float));

    std::vector<float> rr_node_C(rr_nodes.size(), 0.); //Stores the final C

	for (size_t inode = 0; inode < rr_nodes.size(); inode++) {

        //The C may have already been partly initialized (e.g. with metal capacitance)
        rr_node_C[inode] += rr_nodes[inode].C();

		from_rr_type = rr_nodes[inode].type();

		if (from_rr_type == CHANX || from_rr_type == CHANY) {

			for (iedge = 0; iedge < rr_nodes[inode].num_edges(); iedge++) {

				to_node = rr_nodes[inode].edge_sink_node(iedge);
				to_rr_type = rr_nodes[to_node].type();

				if (to_rr_type == CHANX || to_rr_type == CHANY) {

					switch_index = rr_nodes[inode].edge_switch(iedge);
					Cin = rr_switch_inf[switch_index].Cin;
					Cout = rr_switch_inf[switch_index].Cout;
					buffered = rr_switch_inf[switch_index].buffered();

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
						if (rr_nodes[to_node].direction() == BI_DIRECTION) {
							/* For multiple-driver architectures the output capacitance can
							 * be added now since each edge is actually a driver */
							rr_node_C[to_node] += Cout;
						}
						isblock = seg_index_of_sblock(inode, to_node);
						buffer_Cin[isblock] = max(buffer_Cin[isblock], Cin);
					}

				}
				/* End edge to CHANX or CHANY node. */
				else if (to_rr_type == IPIN) {

					if (INCLUDE_TRACK_BUFFERS){
						/* Implements sharing of the track to connection box buffer.
						   Such a buffer exists at every segment of the wire at which
						   at least one logic block input connects. */
						icblock = seg_index_of_cblock(from_rr_type, to_node);
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

			/*   for (iedge=0;iedge<device_ctx.rr_nodes[inode].num_edges();iedge++) {
			 * to_node = device_ctx.rr_nodes[inode].edges[iedge];
			 * if (device_ctx.rr_nodes[to_node].type() == IPIN) {
			 * icblock = seg_index_of_cblock (from_rr_type, to_node);
			 * cblock_counted[icblock] = false;
			 * }
			 * }     */

			if (from_rr_type == CHANX) {
				iseg_low = rr_nodes[inode].xlow();
				iseg_high = rr_nodes[inode].xhigh();
			} else { /* CHANY */
				iseg_low = rr_nodes[inode].ylow();
				iseg_high = rr_nodes[inode].yhigh();
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

			for (iedge = 0; iedge < rr_nodes[inode].num_edges(); iedge++) {
				switch_index = rr_nodes[inode].edge_switch(iedge);
				to_node = rr_nodes[inode].edge_sink_node(iedge);
				to_rr_type = rr_nodes[to_node].type();

				if (to_rr_type != CHANX && to_rr_type != CHANY)
					continue;

				if (rr_nodes[to_node].direction() == BI_DIRECTION) {
					Cout = rr_switch_inf[switch_index].Cout;
					to_node = rr_nodes[inode].edge_sink_node(iedge); /* Will be CHANX or CHANY */
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
	Couts_to_add = (float *) vtr::calloc(rr_nodes.size(), sizeof(float));
	for (size_t inode = 0; inode < rr_nodes.size(); inode++) {
		for (iedge = 0; iedge < rr_nodes[inode].num_edges(); iedge++) {
			switch_index = rr_nodes[inode].edge_switch(iedge);
			to_node = rr_nodes[inode].edge_sink_node(iedge);
			to_rr_type = rr_nodes[to_node].type();
			if (to_rr_type == CHANX || to_rr_type == CHANY) {
				if (rr_nodes[to_node].direction() != BI_DIRECTION) {
					/* Cout was not added in these cases */
					Couts_to_add[to_node] = std::max(Couts_to_add[to_node], rr_switch_inf[switch_index].Cout);

				}
			}
		}
	}
	for (size_t inode = 0; inode < rr_nodes.size(); inode++) {
		rr_node_C[inode] += Couts_to_add[inode];
	}

    //Create the final flywieghted t_rr_rc_data
	for (size_t inode = 0; inode < rr_nodes.size(); inode++) {
        rr_nodes[inode].set_rc_index(find_create_rr_rc_data(rr_nodes[inode].R(), rr_node_C[inode]));
    }

	free(Couts_to_add);
	free(cblock_counted);
	free(buffer_Cin);
}

void t_rr_graph::load_rr_index_segments(const int num_segment) {
    int iseg, i, index;

    for (i = SOURCE_COST_INDEX; i <= IPIN_COST_INDEX; i++) {
        rr_indexed_data[i].seg_index = OPEN;
    }

    /* X-directed segments. */
    for (iseg = 0; iseg < num_segment; iseg++) {
        index = CHANX_COST_INDEX_START + iseg;
        rr_indexed_data[index].seg_index = iseg;
    }
    /* Y-directed segments. */
    for (iseg = 0; iseg < num_segment; iseg++) {
        index = CHANX_COST_INDEX_START + num_segment + iseg;
        rr_indexed_data[index].seg_index = iseg;
    }
}

/* Allocates the device_ctx.rr_indexed_data array and loads it with appropriate values. *
 * It currently stores the segment type (or OPEN if the index doesn't        *
 * correspond to an CHANX or CHANY type), the base cost of nodes of that     *
 * type, and some info to allow rapid estimates of time to get to a target   *
 * to be computed by the router.                                             *
 *
 * Right now all SOURCES have the same base cost; and similarly there's only *
 * one base cost for each of SINKs, OPINs, and IPINs (four total).  This can *
 * be changed just by allocating more space in the array below and changing  *
 * the cost_index values for these rr_nodes, if you want to make some pins   *
 * etc. more expensive than others.  I give each segment type in an          *
 * x-channel its own cost_index, and each segment type in a y-channel its    *
 * own cost_index.                                                           */
void t_rr_graph::alloc_and_load_rr_indexed_data(const std::vector<t_segment_inf>& segment_inf,
                                                int wire_to_ipin_switch,
                                                enum e_base_cost_type base_cost_type) {

    int iseg, length, i, index;

    auto& device_ctx = g_vpr_ctx.mutable_device();
    int num_segment = segment_inf.size();
    int num_rr_indexed_data = CHANX_COST_INDEX_START + (2 * num_segment);
    device_ctx.rr_indexed_data.resize(num_rr_indexed_data);

    /* For rr_types that aren't CHANX or CHANY, base_cost is valid, but most     *
     * * other fields are invalid.  For IPINs, the T_linear field is also valid;   *
     * * all other fields are invalid.  For SOURCES, SINKs and OPINs, all fields   *
     * * other than base_cost are invalid. Mark invalid fields as OPEN for safety. */

    for (i = SOURCE_COST_INDEX; i <= IPIN_COST_INDEX; i++) {
        device_ctx.rr_indexed_data[i].ortho_cost_index = OPEN;
        device_ctx.rr_indexed_data[i].seg_index = OPEN;
        device_ctx.rr_indexed_data[i].inv_length = OPEN;
        device_ctx.rr_indexed_data[i].T_linear = OPEN;
        device_ctx.rr_indexed_data[i].T_quadratic = OPEN;
        device_ctx.rr_indexed_data[i].C_load = OPEN;
    }
    device_ctx.rr_indexed_data[IPIN_COST_INDEX].T_linear =
            device_ctx.rr_switch_inf[wire_to_ipin_switch].Tdel;

    /* X-directed segments. */
    for (iseg = 0; iseg < num_segment; iseg++) {
        index = CHANX_COST_INDEX_START + iseg;

        if ((index + num_segment) >= (int) device_ctx.rr_indexed_data.size()) {
            device_ctx.rr_indexed_data[index].ortho_cost_index = index;
        } else {
            device_ctx.rr_indexed_data[index].ortho_cost_index = index + num_segment;
        }

        if (segment_inf[iseg].longline)
            length = device_ctx.grid.width();
        else
            length = std::min<int>(segment_inf[iseg].length, device_ctx.grid.width());

        device_ctx.rr_indexed_data[index].inv_length = 1. / length;
        device_ctx.rr_indexed_data[index].seg_index = iseg;
    }
    load_rr_indexed_data_T_values(CHANX_COST_INDEX_START, num_segment, CHANX,
            nodes_per_chan, L_rr_node_indices);

    /* Y-directed segments. */
    for (iseg = 0; iseg < num_segment; iseg++) {
        index = CHANX_COST_INDEX_START + num_segment + iseg;

        if((index - num_segment) < CHANX_COST_INDEX_START) {
            device_ctx.rr_indexed_data[index].ortho_cost_index = index;
        } else {
            device_ctx.rr_indexed_data[index].ortho_cost_index = index - num_segment;
        }

        if (segment_inf[iseg].longline)
            length = device_ctx.grid.height();
        else
            length = std::min<int>(segment_inf[iseg].length, device_ctx.grid.height());

        device_ctx.rr_indexed_data[index].inv_length = 1. / length;
        device_ctx.rr_indexed_data[index].seg_index = iseg;
    }
    load_rr_indexed_data_T_values((CHANX_COST_INDEX_START + num_segment),
            num_segment, CHANY, nodes_per_chan, L_rr_node_indices);

    load_rr_indexed_data_base_costs(nodes_per_chan, L_rr_node_indices,
            base_cost_type);
}

int t_rr_graph::get_max_chan_width() {
    int max_chan_width = (rr_graph_type ? 1 : chan_width.max);
    VTR_ASSERT(max_chan_width > 0);

    return max_chan_width;
}

void t_rr_graph::load_rr_indexed_data_T_values(const DeviceGrid& grid, int index_start,
                                               int num_indices_to_load, t_rr_type rr_type) {

    /* Loads the average propagation times through segments of each index type  *
     * for either all CHANX segment types or all CHANY segment types.  It does  *
     * this by looking at all the segments in one channel in the middle of the  *
     * array and averaging the R and C values of all segments of the same type  *
     * and using them to compute average delay values for this type of segment. */

    int itrack, inode, cost_index;
    float *C_total, *R_total; /* [0..device_ctx.rr_indexed_data.size() - 1] */
    double *switch_R_total, *switch_T_total; /* [0..device_ctx.rr_indexed_data.size() - 1] */
    short *switches_buffered;
    int *num_nodes_of_index; /* [0..device_ctx.rr_indexed_data.size() - 1] */
    float Rnode, Cnode, Rsw, Tsw;

    num_nodes_of_index = (int *) vtr::calloc(rr_indexed_data.size(), sizeof (int));
    C_total = (float *) vtr::calloc(rr_indexed_data.size(), sizeof (float));
    R_total = (float *) vtr::calloc(rr_indexed_data.size(), sizeof (float));

    /* August 2014: Not all wire-to-wire switches connecting from some wire segment will
       necessarily have the same delay. i.e. a mux with less inputs will have smaller delay
       than a mux with a greater number of inputs. So to account for these differences we will
       get the average R/Tdel values by first averaging them for a single wire segment (first
       for loop below), and then by averaging this value over all wire segments in the channel
       (second for loop below) */
    switch_R_total = (double *) vtr::calloc(rr_indexed_data.size(), sizeof (double));
    switch_T_total = (double *) vtr::calloc(rr_indexed_data.size(), sizeof (double));
    switches_buffered = (short *) vtr::calloc(rr_indexed_data.size(), sizeof (short));

    /* initialize switches_buffered array */
    for (int i = index_start; i < index_start + num_indices_to_load; i++) {
        switches_buffered[i] = UNDEFINED;
    }

    /* Get average C and R values for all the segments of this type in one      *
     * channel segment, near the middle of the fpga.                            */

    for (itrack = 0; itrack < nodes_per_chan; itrack++) {
        inode = find_average_rr_node_index(grid.width(), grid.height(), rr_type, itrack,
                rr_node_indices);
        if (inode == -1)
            continue;
        cost_index = rr_nodes[inode].cost_index();
        num_nodes_of_index[cost_index]++;
        C_total[cost_index] += rr_nodes[inode].C();
        R_total[cost_index] += rr_nodes[inode].R();

        /* get average switch parameters */
        int num_edges = rr_nodes[inode].num_edges();
        double avg_switch_R = 0;
        double avg_switch_T = 0;
        int num_switches = 0;
        short buffered = UNDEFINED;
        for (int iedge = 0; iedge < num_edges; iedge++) {
            int to_node_index = rr_nodes[inode].edge_sink_node(iedge);
            /* want to get C/R/Tdel of switches that connect this track segment to other track segments */
            if (rr_nodes[to_node_index].type() == CHANX || rr_nodes[to_node_index].type() == CHANY) {
                int switch_index = rr_nodes[inode].edge_switch(iedge);
                avg_switch_R += rr_switch_inf[switch_index].R;
                avg_switch_T += rr_switch_inf[switch_index].Tdel;

                num_switches++;
            }
        }

        if (num_switches == 0) {
            VTR_LOG_WARN( "Track %d had no switches\n", itrack);
            continue;
        }
        VTR_ASSERT(num_switches > 0);

        avg_switch_R /= num_switches;
        avg_switch_T /= num_switches;
        switch_R_total[cost_index] += avg_switch_R;
        switch_T_total[cost_index] += avg_switch_T;

        if (buffered == UNDEFINED) {
            /* this segment does not have any outgoing edges to other general routing wires */
            continue;
        }

        /* need to make sure all wire switches of a given wire segment type have the same 'buffered' value */
        if (switches_buffered[cost_index] == UNDEFINED) {
            switches_buffered[cost_index] = buffered;
        } else {
            if (switches_buffered[cost_index] != buffered) {
                vpr_throw(VPR_ERROR_ARCH, __FILE__, __LINE__,
                        "Expecting all wire-to-wire switches of wire segments with cost index (%d) to have same 'buffered' value (%d), but found segment switch with different 'buffered' value (%d)\n", cost_index, switches_buffered[cost_index], buffered);
            }
        }
    }

    for (cost_index = index_start;
            cost_index < index_start + num_indices_to_load; cost_index++) {

        if (num_nodes_of_index[cost_index] == 0) { /* Segments don't exist. */
            rr_indexed_data[cost_index].T_linear = OPEN;
            rr_indexed_data[cost_index].T_quadratic = OPEN;
            rr_indexed_data[cost_index].C_load = OPEN;
        } else {
            Rnode = R_total[cost_index] / num_nodes_of_index[cost_index];
            Cnode = C_total[cost_index] / num_nodes_of_index[cost_index];
            Rsw = (float) switch_R_total[cost_index] / num_nodes_of_index[cost_index];
            Tsw = (float) switch_T_total[cost_index] / num_nodes_of_index[cost_index];

            if (switches_buffered[cost_index]) {
                rr_indexed_data[cost_index].T_linear = Tsw + Rsw * Cnode
                        + 0.5 * Rnode * Cnode;
                rr_indexed_data[cost_index].T_quadratic = 0.;
                rr_indexed_data[cost_index].C_load = 0.;
            } else { /* Pass transistor */
                rr_indexed_data[cost_index].C_load = Cnode;

                /* See Dec. 23, 1997 notes for deriviation of formulae. */

                rr_indexed_data[cost_index].T_linear = Tsw + 0.5 * Rsw * Cnode;
                rr_indexed_data[cost_index].T_quadratic = (Rsw + Rnode) * 0.5
                        * Cnode;
            }
        }
    }

    free(num_nodes_of_index);
    free(C_total);
    free(R_total);
    free(switch_R_total);
    free(switch_T_total);
    free(switches_buffered);
}

void t_rr_graph::load_rr_indexed_data_base_costs(enum e_base_cost_type base_cost_type) {

    /* Loads the base_cost member of device_ctx.rr_indexed_data according to the specified *
     * base_cost_type.                                                          */

    float delay_normalization_fac;
    size_t index;

    if (base_cost_type == DEMAND_ONLY) {
        delay_normalization_fac = 1.;
    } else {
        delay_normalization_fac = get_delay_normalization_fac(nodes_per_chan, L_rr_node_indices);
    }

    rr_indexed_data[SOURCE_COST_INDEX].base_cost = delay_normalization_fac;
    rr_indexed_data[SINK_COST_INDEX].base_cost = 0.;
    rr_indexed_data[OPIN_COST_INDEX].base_cost = delay_normalization_fac;
    rr_indexed_data[IPIN_COST_INDEX].base_cost = 0.95 * delay_normalization_fac;

    auto rr_segment_counts = count_rr_segment_types();
    size_t total_segments = std::accumulate(rr_segment_counts.begin(), rr_segment_counts.end(), 0u);

    /* Load base costs for CHANX and CHANY segments */

    //Future Work: Since we can now have wire types which don't connect to IPINs,
    //             perhaps consider lowering cost of wires which connect to IPINs
    //             so they get explored earlier (same rational as lowering IPIN costs)

    for (index = CHANX_COST_INDEX_START; index < rr_indexed_data.size(); index++) {

        if (base_cost_type == DELAY_NORMALIZED || base_cost_type == DEMAND_ONLY) {
            rr_indexed_data[index].base_cost = delay_normalization_fac;

        } else if (base_cost_type == DELAY_NORMALIZED_LENGTH) {
            rr_indexed_data[index].base_cost = delay_normalization_fac / rr_indexed_data[index].inv_length;  

        } else if (base_cost_type == DELAY_NORMALIZED_FREQUENCY) {
            int seg_index = rr_indexed_data[index].seg_index;
            float freq_fac = float(rr_segment_counts[seg_index]) / total_segments;

            rr_indexed_data[index].base_cost = delay_normalization_fac / freq_fac;

        } else if (base_cost_type == DELAY_NORMALIZED_LENGTH_FREQUENCY) {
            int seg_index = rr_indexed_data[index].seg_index;
            float freq_fac = float(rr_segment_counts[seg_index]) / total_segments;

            //Base cost = delay_norm / (len * freq)
            //device_ctx.rr_indexed_data[index].base_cost = delay_normalization_fac / ((1. / device_ctx.rr_indexed_data[index].inv_length) * freq_fac);

            //Base cost = (delay_norm * len) * (1 + (1-freq))
            rr_indexed_data[index].base_cost = (delay_normalization_fac / rr_indexed_data[index].inv_length) * (1 + (1 - freq_fac));

        } else {
            VPR_THROW(VPR_ERROR_ROUTE, "Unrecognized base cost type");
        }
    }

    /* Save a copy of the base costs -- if dynamic costing is used by the     *
     * router, the base_cost values will get changed all the time and being   *
     * able to restore them from a saved version is useful.                   */

    for (index = 0; index < rr_indexed_data.size(); index++) {
        rr_indexed_data[index].saved_base_cost = rr_indexed_data[index].base_cost;
    }
}

/* Allocates space for the global device_ctx.rr_switch_inf variable and returns the
   number of rr switches that were allocated */
void t_rr_graph_alloc_rr_switch_inf(t_arch_switch_fanin& arch_switch_fanins) {

    int num_rr_switches = 0;
    {
        //Collect the fan-in per switch type for each node in the graph
        //
        //Note that since we don't store backward edge info in the RR graph we need to walk
        //the whole graph to get the per-switch-type fanin info
        std::vector<vtr::flat_map<int,int>> inward_switch_inf(rr_nodes.size()); //[to_node][arch_switch] -> fanin
        for (size_t inode = 0; inode < rr_nodes.size(); ++inode) {
            for (auto iedge : rr_nodes[inode].edges()) {
                int iswitch = rr_nodes[inode].edge_switch(iedge);
                int to_node = rr_nodes[inode].edge_sink_node(iedge);

                if (inward_switch_inf[to_node].count(iswitch) == 0) {
                    inward_switch_inf[to_node][iswitch] = 0;
                }
                inward_switch_inf[to_node][iswitch]++;
            }
        }

        //Record the unique switch type/fanin combinations
        for (size_t inode = 0; inode < rr_nodes.size(); ++inode) {
            for (auto& switch_fanin : inward_switch_inf[inode]) {
                int iswitch, fanin;
                std::tie(iswitch, fanin) = switch_fanin;

                if (arch_switch_inf[iswitch].fixed_Tdel()) {
                    //If delay is independent of fanin drop the unique fanin info
                    fanin = UNDEFINED;
                }

                if (arch_switch_fanins[iswitch].count(fanin) == 0) { //New fanin for this switch
                    arch_switch_fanins[iswitch][fanin] = num_rr_switches++; //Assign it a unique index
                }
            }
        }
    }

    /* allocate space for the rr_switch_inf array */
    rr_switch_inf.resize(num_rr_switches);
}


/* Allocates and loads the global rr_switch_inf array based on the global
   arch_switch_inf array and the fan-ins used by the rr nodes.
   Also changes switch indices of rr_nodes to index into rr_switch_inf
   instead of arch_switch_inf.

   Returns the number of rr switches created.
   Also returns, through a pointer, the index of a representative ipin cblock switch.
        - Currently we're not allowing a designer to specify an ipin cblock switch with
          multiple fan-ins, so there's just one of these switches in the device_ctx.rr_switch_inf array.
          But in the future if we allow this, we can return an index to a representative switch

   The rr_switch_inf switches are derived from the arch_switch_inf switches
   (which were read-in from the architecture file) based on fan-in. The delays of
   the rr switches depend on their fan-in, so we first go through the rr_nodes
   and count how many different fan-ins exist for each arch switch.
   Then we create these rr switches and update the switch indices
   of rr_nodes to index into the rr_switch_inf array. */
void t_rr_graph::alloc_and_load_rr_switch_inf(const float R_minW_nmos, const float R_minW_pmos,
                                              const int wire_to_arch_ipin_switch, int *wire_to_rr_ipin_switch) {
    /* we will potentially be creating a couple of versions of each arch switch where
     * each version corresponds to a different fan-in. We will need to fill device_ctx.rr_switch_inf
     * with this expanded list of switches.
     *
     * To do this we will use arch_switch_fanins, which is indexed as:
     *      arch_switch_fanins[i_arch_switch][fanin] -> new_switch_id
     */
    t_arch_switch_fanin arch_switch_fanins(num_arch_switches);

    /* Determine what the different fan-ins are for each arch switch, and also
       how many entries the rr_switch_inf array should have */
    alloc_rr_switch_inf(arch_switch_fanins);

    /* create the rr switches. also keep track of, for each arch switch, what index of the rr_switch_inf
       array each version of its fanin has been mapped to */
    load_rr_switch_inf(num_arch_switches, R_minW_nmos, R_minW_pmos, arch_switch_fanins);

    /* next, walk through rr nodes again and remap their switch indices to rr_switch_inf */
    remap_rr_node_switch_indices(arch_switch_fanins);

    /* now we need to set the wire_to_rr_ipin_switch variable which points the detailed routing architecture
       to the representative ipin cblock switch. currently we're not allowing the specification of an ipin cblock switch
       with multiple fan-ins, so right now there's just one. May change in the future, in which case we'd need to
       return a representative switch */
    if (arch_switch_fanins[wire_to_arch_ipin_switch].count(UNDEFINED)) {
        /* only have one ipin cblock switch. OK. */
        (*wire_to_rr_ipin_switch) = arch_switch_fanins[wire_to_arch_ipin_switch][UNDEFINED];
    } else if (arch_switch_fanins[wire_to_arch_ipin_switch].size() != 0) {
        vpr_throw(VPR_ERROR_ARCH, __FILE__, __LINE__,
                "Not currently allowing an ipin cblock switch to have multiple fan-ins");
    } else {
        //This likely indicates that no connection block has been constructed, indicating significant issues with
        //the generated RR graph.
        //
        //Instead of throwing an error we issue a warning. This means that check_rr_graph() etc. will run to give more information
        //and allow graphics to be brought up for users to debug their architectures.
        (*wire_to_rr_ipin_switch) = OPEN;
        VTR_LOG_WARN(
                "No switch found for the ipin cblock in RR graph. Check if there is an error in arch file, or if no connection blocks are being built in RR graph\n");
    }

}


/* A utility routine to dump the contents of the routing resource graph   *
 * (everything -- connectivity, occupancy, cost, etc.) into a file.  Used *
 * only for debugging.                                                    */
void t_rr_graph::dump_rr_graph(const char *file_name) {

    FILE *fp = vtr::fopen(file_name, "w");

    for (size_t inode = 0; inode < rr_nodes.size(); ++inode) {
        print_rr_node(fp, rr_nodes, inode);
        fprintf(fp, "\n");
    }

    fclose(fp);
}

