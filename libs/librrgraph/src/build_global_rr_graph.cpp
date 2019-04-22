#include "vtr_time.h" 

#include "rr_graph.h"
#include "rr_graph_error.h"
#include "rr_graph_builder_opts.h"
#include "device_types.h"
#include "clb2clb_directs.h"
#include "seg_details.h"
#include "chan_width.h"

void RRGraph::build_global_rr_graph(const t_rr_graph_builder_opts builder_opts,
                                    const t_arch& arch, 
                                    const DeviceGrid& device_grid,
                                    const DeviceTypes& device_types) {

  vtr::ScopedStartFinishTimer timer("Build global routing resource graph");

  /* FIXME: this causes werid errors: Reset warning flag */
  /* int *Warnings = RR_GRAPH_NO_WARN; */

  /* Update the type of this rr_graph */
  this->set_type(GRAPH_GLOBAL);
  this->set_directionality(BI_DIRECTIONAL);

  /* Decode the graph_type */
  bool is_global_graph = true;
  bool use_full_seg_groups = false;

  /* Global routing uses a single longwire track */
  int max_chan_width = 1;

  /* Allocate direct connections in routing architectures */
  /* FIXME: this function is used during placement and routing, should be a common class */
  Clb2ClbDirects clb2clb_directs;
  clb2clb_directs.init(arch.num_directs, arch.Directs, 
                       device_types,
                       builder_opts.delayless_switch);

  t_chan_width chan_width;
  chan_width.init(device_grid, builder_opts.chan_width_fac, arch.Chans);

  /* START SEG_DETAILS */
  int num_seg_details = 0;
  t_seg_details *seg_details = nullptr;

  /* Sets up a single unit length segment type for global routing. */
  seg_details = alloc_and_load_global_route_seg_details(builder_opts.global_route_switch, 
                                                        &num_seg_details);

  /* END SEG_DETAILS */

  /* START CHAN_DETAILS */
  t_chan_details chan_details_x;
  t_chan_details chan_details_y;


  alloc_and_load_chan_details(device_types, device_grid, &chan_width,
                              builder_opts.trim_empty_channels, builder_opts.trim_obs_channels,
                              num_seg_details, seg_details,
                              chan_details_x, chan_details_y);

//#ifdef VERBOSE
//  if ( true == builder_opts.echo_chan_details ) {
//    /* Assert: must have a valid filename if echo is enabled */
//    VTR_ASSERT_MSG( (  (true == builder_opts.echo_chan_details) 
//                    && (NULL != builder_opts.chan_details_echo_filename)),
//                    "Echo file has been enabled but output file name is invalid");
//
//    dump_chan_details(chan_details_x, chan_details_y, max_chan_width, device_grid,
//                      builder_opts.chan_details_echo_filename);
//  }
//#endif
  /* END CHAN_DETAILS */

  /* START FC */
  /* Determine the actual value of Fc */
  std::vector<vtr::Matrix<int>> Fc_in; /* [0..device_ctx.num_block_types-1][0..num_pins-1][0..num_segments-1] */
  std::vector<vtr::Matrix<int>> Fc_out; /* [0..device_ctx.num_block_types-1][0..num_pins-1][0..num_segments-1] */

  /* get maximum number of pins across all blocks */
  int max_pins = device_types.get_types(0)->num_pins;
  for (int i = 1; i < device_types.get_num_types(); ++i) {
    if (device_types.get_types(i)->num_pins > max_pins) {
      max_pins = device_types.get_types(i)->num_pins;
    }
  }

  /* get the number of 'sets' for each segment type -- unidirectial architectures have two tracks in a set, bidirectional have one */
  int total_sets = max_chan_width;

  int *sets_per_seg_type = get_seg_track_counts(total_sets, arch.Segments, use_full_seg_groups);

  //All pins can connect during global routing
  auto ones = vtr::Matrix<int>({size_t(max_pins), arch.Segments.size()}, 1);
  Fc_in = std::vector<vtr::Matrix<int>>(device_types.get_num_types(), ones);
  Fc_out = std::vector<vtr::Matrix<int>>(device_types.get_num_types(), ones);

//  auto perturb_ipins = alloc_and_load_perturb_ipins(arch.num_types, arch.Segments.size(),
//                                                    sets_per_seg_type, Fc_in, Fc_out, 
//                                                    this->directionality());
//  /* END FC */
//
//  /* Alloc node lookups, count nodes, alloc rr nodes */
//  int num_rr_nodes = 0;
//
//  /* Allocate node look-up in the rr_graph */
//  t_rr_node_indices rr_node_indices = alloc_and_load_rr_node_indices(max_chan_width, device_grid,
//                                                                     &num_rr_nodes, 
//                                                                     chan_details_x, chan_details_y);
//  /* Allocate nodes in the rr_graph */
//  nodes_ids_.resize(num_rr_nodes);
//
//  /* These are data structures used by the the unidir opin mapping. They are used
//     to spread connections evenly for each segment type among the available
//     wire start points */
//  vtr::NdMatrix<int, 3> Fc_xofs({
//                                 grid.height() - 1,
//                                 grid.width() - 1,
//                                 segment_inf.size()
//                                },
//                                0); //[0..grid.height()-2][0..grid.width()-2][0..num_seg_types-1]
//  vtr::NdMatrix<int, 3> Fc_yofs({
//                                grid.width() - 1,
//                                grid.height() - 1,
//                                segment_inf.size() 
//                                },
//                                0); //[0..grid.width()-2][0..grid.height()-2][0..num_seg_types-1]
//
//  /* START SB LOOKUP */
//  /* Alloc and load the switch block lookup */
//  vtr::NdMatrix<std::vector<int>, 3> switch_block_conn;
//  t_sb_connection_map *sb_conn_map = nullptr; //for custom switch blocks
//
//  //We are careful to use a single seed each time build_rr_graph is called
//  //to initialize the random number generator (RNG) which is (potentially)
//  //used when creating custom switchblocks. This ensures that build_rr_graph()
//  //is deterministic -- always producing the same RR graph.
//  constexpr unsigned SWITCHPOINT_RNG_SEED = 1;
//  vtr::RandState switchpoint_rand_state =  SWITCHPOINT_RNG_SEED;
//
//  switch_block_conn = alloc_and_load_switch_block_conn(1, SUBSET, 3);
//  /* END SB LOOKUP */
//
//  /* START IPIN MAP */
//  /* Create ipin map lookups */
//
//  t_pin_to_track_lookup ipin_to_track_map(builder_opts.num_types); /* [0..device_ctx.num_block_types-1][0..num_pins-1][0..width][0..height][0..3][0..Fc-1] */
//  t_track_to_pin_lookup track_to_pin_lookup(builder_opts.num_types); /* [0..device_ctx.num_block_types-1][0..max_chan_width-1][0..width][0..height][0..3] */
//
//  for (int itype = 0; itype < builder_opts.num_types; ++itype) {
//
//    ipin_to_track_map[itype] = alloc_and_load_pin_to_track_map(RECEIVER,
//                                 Fc_in[itype], &(builder_opts.types[itype]), perturb_ipins[itype], 
//                                 this->directionality(),
//                                 arch.Segments.size(), sets_per_seg_type);
//
//    track_to_pin_lookup[itype] = alloc_and_load_track_to_pin_lookup(
//                                   ipin_to_track_map[itype], Fc_in[itype], 
//                                   builder_opts.types[itype].width, builder_opts.types[itype].height,
//                                   builder_opts.types[itype].num_pins, max_chan_width, 
//                                   arch.Segments.size());
//  }
//  /* END IPIN MAP */
//
//  /* START OPIN MAP */
//  /* Create opin map lookups */
//  t_pin_to_track_lookup opin_to_track_map(builder_opts.num_types); /* [0..device_ctx.num_block_types-1][0..num_pins-1][0..width][0..height][0..3][0..Fc-1] */
// 
//  /* END OPIN MAP */
//
//  bool Fc_clipped = false;
//  alloc_and_load_rr_graph(nodes_ids.size(), device_ctx.rr_nodes, segment_inf.size(),
//            chan_details_x, chan_details_y,
//            track_to_pin_lookup, opin_to_track_map,
//            switch_block_conn, sb_conn_map, grid, Fs, unidir_sb_pattern,
//            Fc_out, Fc_xofs, Fc_yofs, device_ctx.rr_node_indices,
//            max_chan_width,
//            nodes_per_chan,
//            wire_to_arch_ipin_switch,
//            delayless_switch,
//            directionality,
//            &Fc_clipped,
//            directs, num_directs, clb_to_clb_directs,
//            is_global_graph);
//
//  /* Update rr_nodes capacities if global routing */
//  for (size_t inode = 0; inode < node_ids_.size(); ++inode) {
//    if (CHANX == this->node_type(inode)) {
//      size_t ylow = this->node_ylow(i);
//      this->set_node_capacity(nodes_per_chan.x_list[ylow]);
//      continue;
//    }
//    if (CHANY == this->node_type(inode)) {
//      size_t xlow = this->node_xlow(i);
//      this->set_node_capacity(nodes_per_chan.y_list[xlow]);
//    }
//  }
//
//  /* Allocate and load routing resource switches, which are derived from the switches from the architecture file,
//   * based on their fanin in the rr graph. This routine also adjusts the rr nodes to point to these new rr switches 
//   */
//  alloc_and_load_rr_switch_inf(num_arch_switches, R_minW_nmos, R_minW_pmos, wire_to_arch_ipin_switch, wire_to_rr_ipin_switch);
//
//  //Partition the rr graph edges for efficient access to configurable/non-configurable
//  //edge subsets. Must be done after RR switches have been allocated
//  partition_rr_graph_edges(device_ctx);
//
//  //Save the channel widths for the newly constructed graph
//  /* FIXME: this should be move to somewhere outside this library!!!
//   * device_ctx.chan_width = nodes_per_chan;
//   */
//
//  rr_graph_externals(arch.Segments, max_chan_width,
//                     *wire_to_rr_ipin_switch, base_cost_type);
//
//
//  check_rr_graph(this->type(), device_grid, builder_opts.types);
//
//  /* Free all temp structs */
//  if (seg_details) {
//      delete[] seg_details;
//      seg_details = nullptr;
//  }
//  if (!chan_details_x.empty() || !chan_details_y.empty()) {
//      free_chan_details(chan_details_x, chan_details_y);
//  }
//  if (sb_conn_map) {
//      free_switchblock_permutations(sb_conn_map);
//      sb_conn_map = nullptr;
//  }
//  if (sets_per_seg_type) {
//      free(sets_per_seg_type);
//      sets_per_seg_type = nullptr;
//  }
//
//  free_type_track_to_pin_map(track_to_pin_lookup, types, max_chan_width);

  return; 
}

