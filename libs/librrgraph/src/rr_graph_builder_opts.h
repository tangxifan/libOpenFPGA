#ifndef RR_GRAPH_BUILDER_OPTS
#define RR_GRAPH_BUILDER_OPTS

enum e_base_cost_type {
	DELAY_NORMALIZED, 
	DELAY_NORMALIZED_LENGTH, 
	DELAY_NORMALIZED_FREQUENCY, 
	DELAY_NORMALIZED_LENGTH_FREQUENCY, 
    DEMAND_ONLY
};

enum e_clock_modeling {
    IDEAL_CLOCK,      //Treat the clock pins as ideal (i.e. not routed)
    ROUTED_CLOCK,     //Treat the clock pins as normal nets (i.e. routed)
    DEDICATED_NETWORK //Connect clock pins to a dedicated clock network
};

enum class e_router_lookahead {
    CLASSIC, //VPR's classic lookahead (assumes uniform wire types)
    MAP,     //Lookahead considering different wire types (see Oleg Petelin's MASc Thesis)
    NO_OP    //A no-operation lookahead which always returns zero
};

struct t_rr_graph_builder_opts {
  /* Copies from router opts, to keep things separated */
  bool trim_empty_channels;
  bool trim_obs_channels;
  enum e_base_cost_type base_cost_type;
  enum e_clock_modeling clock_modeling; //How clock pins and nets should be handled
  e_router_lookahead lookahead_type;

  /* Cofactor used for building channel width */
  int chan_width_fac;

  /* Switch information, should be a copy from t_det_arch */
  short global_route_switch;
  short delayless_switch;
  
  /* Flags to echo any internal data structures */
  bool echo_chan_details;
  char* chan_details_echo_filename;
};

#endif


