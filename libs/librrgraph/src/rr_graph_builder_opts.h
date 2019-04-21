#ifndef RR_GRAPH_BUILDER_OPTS
#define RR_GRAPH_BUILDER_OPTS

struct t_rr_graph_builder_opts {
  /* TODO: To simplify the input of rr_graph builders,
   * The option data structure includes other data structures.   
   * Ideally, they should come from major input data structures:
   * t_arch, device_grid, etc.
   */ 
  int numTypes;
  t_type_descriptor* types = NULL;

  /* Copies from router opts, to keep things separated */
  bool trim_empty_channels;
  bool trim_obs_channels;
  enum e_base_cost_type base_cost_type;
  enum e_clock_modeling clock_modeling; //How clock pins and nets should be handled
  e_router_lookahead lookahead_type;

  /* Cofactor used for building channel width */
  int chan_width_fac;

  /* Switch information, should be a copy from t_det_arch */
  short delayless_switch;
  
  /* Flags to echo any internal data structures */
  bool echo_chan_details;
  char* chan_details_echo_filename;
};

#endif


