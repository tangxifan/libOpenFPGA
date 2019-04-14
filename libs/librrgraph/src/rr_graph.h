#ifndef RR_GRAPH_H
#define RR_GRAPH_H

/* A Context is collection of state relating to a particular part of VPR
 *
 * This is a base class who's only purpose is to disable copying of contexts.
 * This ensures that attempting to use a context by value (instead of by reference)
 * will result in a compilation error.
 *
 * No data or member functions should be defined in this class!
 */
struct Context {
    //Contexts are non-copyable
    Context() = default;
    Context(Context&) = delete;
    Context& operator=(Context&) = delete;
    virtual ~Context() = default;
};

enum e_rr_graph_type {
    GRAPH_GLOBAL, /* One node per channel with wire capacity > 1 and full connectivity */
    GRAPH_BIDIR, /* Detailed bidirectional graph */
    GRAPH_UNIDIR, /* Detailed unidir graph, untilable */
    /* RESEARCH TODO: Get this option debugged */
    GRAPH_UNIDIR_TILEABLE /* Detail unidir graph with wire groups multiples of 2*L */
};
typedef enum e_rr_graph_type t_rr_graph_type;

/* Channel width data */
struct t_chan_width {
	int max = 0;
	int x_max = 0;
	int y_max = 0;
	int x_min = 0;
	int y_min = 0;
    std::vector<int> x_list;
	std::vector<int> y_list;
};

typedef std::vector<std::vector<std::vector<std::vector<std::vector<int>>>>> t_rr_node_indices; //[0..num_rr_types-1][0..grid_width-1][0..grid_height-1][0..NUM_SIDES-1][0..max_ptc-1]

typedef std::vector<std::map<int,int>> t_arch_switch_fanin;

/* Class for a Routing Resource Graph
 * 
 */
class t_rr_graph : public Context {
  public:
    /* Methods to create/free/access/modify each member */

    /* Constructor for routing channels */
    void init_chan(const DeviceGrid& grid, int cfactor, t_chan_width_dist chan_width_dist) {

    /* Useful functions to get a wanted rr_node */
    /* Generic function to get the index of a rr_node */
    int get_rr_node_index(const DeviceGrid& grid, int x, int y, t_rr_type rr_type, int ptc, e_side side);
    /* Function to get the index of a rr_node whose type is CHANX or CHANY */
    int get_chan_rr_node_index(const DeviceGrid& grid, int x, int y, t_rr_type chan_rr_type, int ptc);
    /* Function to get the index of a rr_node whose type is OPIN or PIN */
    int get_grid_pin_rr_node_index(const DeviceGrid& grid, int x, int y, t_rr_type pin_rr_type, int ptc, e_side side);
    /* Build a rr_graph */
    void build_rr_graph(const t_graph_type graph_type, const t_arch arch);
    /* Constructor for the look-up table of rr_node: rr_indices */
    void alloc_rr_node_indices(const DeviceGrid& grid);
    /* Load routing channel information for the look-up table of rr_node */
    void load_chan_rr_indices(const int max_chan_width, const int chan_len,
                              const int num_chans, const t_rr_type type,
                              const t_chan_details& chan_details, int *index);
    /* Load logic block information to the look-up table of rr_node */
    void load_block_rr_indices(const DeviceGrid& grid, int* index);

    void load_rr_index_segments(const int num_segment);

    void alloc_and_load_rr_indexed_data(const std::vector<t_segment_inf>& segment_inf,
                                        int wire_to_ipin_switch,
                                        enum e_base_cost_type base_cost_type);

    void load_rr_indexed_data_T_values(const DeviceGrid& grid, int index_start,
                                       int num_indices_to_load, t_rr_type rr_type);

    void load_rr_indexed_data_base_costs(enum e_base_cost_type base_cost_type);

    void alloc_and_load_rr_switch_inf(const float R_minW_nmos, const float R_minW_pmos,
                                        const int wire_to_arch_ipin_switch, int *wire_to_rr_ipin_switch);

    void load_rr_switch_inf(const float R_minW_nmos, const float R_minW_pmos) {

    void load_rr_switch_inf(const int num_arch_switches, const float R_minW_nmos, const float R_minW_pmos) {

    void load_rr_switch_from_arch_switch(int arch_switch_idx, int rr_switch_idx,
                                         int fanin, const float R_minW_nmos, const float R_minW_pmos) {

    void remap_rr_node_switch_indices();

    /* Partition the rr graph edges for efficient access to configurable/non-configurable
     * edge subsets. Must be done after RR switches have been allocated
     */
    void partition_rr_graph_edges();

    void add_rr_graph_C_from_switches(float C_ipin_cblock);

    /* Check if the rr_graph is correct after allocation */
    void check_rr_graph(const t_graph_type graph_type,
                        const DeviceGrid& grid,
                        const t_type_ptr types);
    /* Output a rr_graph into a file */
    void dump_rr_graph(const char *file_name);
  public: 
    /* Basic data read/write function */
    t_rr_graph_type type() const { return type_; }
    t_rr_graph_type mutable_type {return type_; }

  private: 
    /* Type of rr_graph */
    t_rr_graph_type type_;
  
    /* chan_width is for x|y-directed channels; i.e. between rows */
    t_chan_width chan_width_;
  
    /* Structures to define the routing architecture of the FPGA.           */
    std::vector<t_rr_node> rr_nodes_; /* autogenerated in build_rr_graph */
  
    std::vector<t_rr_indexed_data> rr_indexed_data_; /* [0 .. num_rr_indexed_data-1] */
  
    //Fly-weighted Resistance/Capacitance data for RR Nodes
    std::vector<t_rr_rc_data> rr_rc_data_;
  
    //The indicies of rr nodes of a given type at a specific x,y grid location
    t_rr_node_indices rr_node_indices_; //[0..NUM_RR_TYPES-1][0..grid.width()-1][0..grid.width()-1][0..size-1]
  
    std::vector<t_rr_switch_inf> rr_switch_inf_; /* autogenerated in build_rr_graph based on switch fan-in. [0..(num_rr_switches-1)] */
  
    int num_arch_switches_;
    t_arch_switch_inf *arch_switch_inf_; /* [0..(num_arch_switches-1)] */
  
    /** Attributes for each rr_node.
     * key:     rr_node index
     * value:   map of <attribute_name, attribute_value>
     */
    std::unordered_map<int, t_metadata_dict> rr_node_metadata_;
    /* Attributes for each rr_edge                                             *
     * key:     <source rr_node_index, sink rr_node_index, iswitch>            *
     * iswitch: Index of the switch type used to go from this rr_node to       *
     *          the next one in the routing.  OPEN if there is no next node    *
     *          (i.e. this node is the last one (a SINK) in a branch of the    *
     *          net's routing).                                                *
     * value:   map of <attribute_name, attribute_value>                       */
    std::unordered_map<std::tuple<int, int, short>,
        t_metadata_dict> rr_edge_metadata_;
  
    /*
     * switch_fanin_remap is only used for printing out switch fanin stats (the -switch_stats option)
     * array index: [0..(num_arch_switches-1)];
     * map key: num of all possible fanin of that type of switch on chip
     * map value: remapped switch index (index in rr_switch_inf)
     */
    std::vector<std::map<int, int>> switch_fanin_remap_;
};

#endif
