#ifndef RR_NODE_H
#define RR_NODE_H
#include "side.h"

#include "vtr_range.h"

#include <memory>
#include <cstdint>

/* Uncomment lines below to save some memory, at the cost of debugging ease. */
/*enum e_rr_type {SOURCE, SINK, IPIN, OPIN, CHANX, CHANY}; */
/* typedef short t_rr_type */

/* Type of a routing resource node.  x-directed channel segment,   *
 * y-directed channel segment, input pin to a clb to pad, output   *
 * from a clb or pad (i.e. output pin of a net) and:               *
 * SOURCE:  A dummy node that is a logical output within a block   *
 *          -- i.e., the gate that generates a signal.             *
 * SINK:    A dummy node that is a logical input within a block    *
 *          -- i.e. the gate that needs a signal.                  */
typedef enum e_rr_type : unsigned char {
	SOURCE = 0, SINK, IPIN, OPIN, CHANX, CHANY, INTRA_CLUSTER_EDGE, NUM_RR_TYPES
} t_rr_type;

/* Constant expression to save memory */
constexpr std::array<t_rr_type, NUM_RR_TYPES> RR_TYPES = { {
	SOURCE, SINK, IPIN, OPIN, CHANX, CHANY, INTRA_CLUSTER_EDGE
} };
/* Constant expression to ease print log files */
constexpr std::array<const char*, NUM_RR_TYPES> rr_node_typename { {
	"SOURCE", "SINK", "IPIN", "OPIN", "CHANX", "CHANY", "INTRA_CLUSTER_EDGE"
} };

/* Directionality of a Routing Resource Node (rr_node)
 * Directionality is only applicable to rr_node whose type is 
 * CHANX or CHANY in e_rr_type
 * i.e., routing tracks
 * 1. INC_DIRECTION or DEC_DIRECTION
 *    applicable to rr_node in uni-directional routing architectures:
 *    INC_DIRECTION:
 *    (a) a CHANX goes from left to right
 *    (b) a CHANY goes from bottom to top 
 *    DEC_DIRECTION:
 *    (a) a CHANX goes from right to left
 *    (b) a CHANY goes from top to bottom
 * 2. BI_DIRECTION: 
 *    applicable to rr_node in bi-directional routing architectures
 * 3. NO_DIRECTION:
 *    FIXME
 * 4. NUM_DIRECTIONS:
 *    A quick counter for directionality types
 */
enum e_direction : unsigned char {
	INC_DIRECTION = 0,
    DEC_DIRECTION = 1,
    BI_DIRECTION = 2,
    NO_DIRECTION = 3,
    NUM_DIRECTIONS
};

/* Main structure describing one routing resource node.  Everything in       *
 * this structure should describe the graph -- information needed only       *
 * to store algorithm-specific data should be stored in one of the           *
 * parallel rr_node_* structures.                                            *
 *                                                                           *
 * xlow, xhigh, ylow, yhigh:  Integer coordinates (see route.c for           *
 *       coordinate system) of the ends of this routing resource.            *
 *       xlow = xhigh and ylow = yhigh for pins or for segments of           *
 *       length 1.  These values are used to decide whether or not this      *
 *       node should be added to the expansion heap, based on things         *
 *       like whether it's outside the net bounding box or is moving         *
 *       further away from the target, etc.                                  *
 * type:  What is this routing resource?                                     *
 * ptc_num:  Pin, track or class number, depending on rr_node type.          *
 *           Needed to properly draw.                                        *
 * cost_index: An integer index into the table of routing resource indexed   *
 *             data t_rr_index_data (this indirection allows quick dynamic   *
 *             changes of rr base costs, and some memory storage savings for *
 *             fields that have only a few distinct values).                 *
 * capacity:   Capacity of this node (number of routes that can use it).     *
 * num_edges:  Number of edges exiting this node.  That is, the number       *
 *             of nodes to which it connects.                                *
 * edges[0..num_edges-1]:  Array of indices of the neighbours of this        *
 *                         node.                                             *
 * switches[0..num_edges-1]:  Array of switch indexes for each of the        *
 *                            edges leaving this node.                       *
 *                                                                           *
 * direction: if the node represents a track, this field                     *
 *            indicates the direction of the track. Otherwise                *
 *            the value contained in the field should be                     *
 *            ignored.                                                       *
 * side: The side of a grid location where an IPIN or OPIN is located.       *
 *       This field is valid only for IPINs and OPINs and should be ignored  *
 *       otherwise.                                                          */

class t_rr_node {
    public: //Types

        //An iterator that dereferences to an edge index
        //
        //Used inconjunction with vtr::Range to return ranges of edge indices
        class edge_idx_iterator : public std::iterator<std::bidirectional_iterator_tag, short> {
            public:
            edge_idx_iterator(value_type init): value_(init) {}
            iterator operator++() { value_ += 1; return *this; }
            iterator operator--() { value_ -= 1; return *this; }
            reference operator*() { return value_; }
            pointer operator->() { return &value_; }

            friend bool operator== (const edge_idx_iterator lhs, const edge_idx_iterator rhs) { return lhs.value_ == rhs.value_; }
            friend bool operator!= (const edge_idx_iterator lhs, const edge_idx_iterator rhs) { return !(lhs == rhs); }

            private:
                value_type value_;
        };

        typedef vtr::Range<edge_idx_iterator> edge_idx_range;
    public: //Accessors
        t_rr_type type() const { return type_; }
        const char *type_string() const; /* Retrieve type as a string */

        edge_idx_range edges() const { return vtr::make_range(edge_idx_iterator(0), edge_idx_iterator(num_edges())); }
        edge_idx_range configurable_edges() const { return vtr::make_range(edge_idx_iterator(0), edge_idx_iterator(num_edges() - num_non_configurable_edges())); }
        edge_idx_range non_configurable_edges() const { return vtr::make_range(edge_idx_iterator(num_edges() - num_non_configurable_edges()), edge_idx_iterator(num_edges())); }

        short num_edges() const { return num_edges_; }
        short num_configurable_edges() const { return num_edges() - num_non_configurable_edges(); }
        short num_non_configurable_edges() const { return num_non_configurable_edges_; }

        int edge_sink_node(short iedge) const { VTR_ASSERT_SAFE(iedge < num_edges()); return edges_[iedge].sink_node; }
        short edge_switch(short iedge) const { VTR_ASSERT_SAFE(iedge < num_edges()); return edges_[iedge].switch_id; }

        bool edge_is_configurable(short iedge) const;
        short fan_in() const;

        short xlow() const;
        short ylow() const;
        short xhigh() const;
        short yhigh() const;
        signed short length() const;

        short capacity() const;

        short ptc_num() const;
        short pin_num() const; //Same as ptc_num() but checks that type() is consistent
        short track_num() const; //Same as ptc_num() but checks that type() is consistent
        short class_num() const; //Same as ptc_num() but checks that type() is consistent

        short cost_index() const;
        short rc_index() const;
        e_direction direction() const;
        const char *direction_string() const;

        e_side side() const;
        const char *side_string() const;

        float R() const;
        float C() const;

        bool validate() const;

    public: //Mutators
        void set_type(t_rr_type new_type);

        short add_edge(int sink_node, int iswitch);

        void shrink_to_fit();

        //Partitions all edges so that configurable and non-configurable edges
        //are organized for efficient access.
        //
        //Must be called before configurable_edges(), non_configurable_edges(),
        //num_configurable_edges(), num_non_configurable_edges() to ensure they
        //are correct.
        void partition_edges();


        void set_num_edges(short); //Note will remove any previous edges
        void set_edge_sink_node(short iedge, int sink_node);
        void set_edge_switch(short iedge, short switch_index);

        void set_fan_in(short);

        void set_coordinates(short x1, short y1, short x2, short y2);

        void set_capacity(short);

        void set_ptc_num(short);
        void set_pin_num(short); //Same as set_ptc_num() by checks type() is consistent
        void set_track_num(short); //Same as set_ptc_num() by checks type() is consistent
        void set_class_num(short); //Same as set_ptc_num() by checks type() is consistent


        void set_cost_index(size_t);
        void set_rc_index(short);

        void set_direction(e_direction);
        void set_side(e_side);

    private: //Types
        //The edge information is stored in a structure to economize on the number of pointers held
        //by t_rr_node (to save memory), and is not exposed externally
        struct t_rr_edge {
            int sink_node = -1; //The ID of the sink RR node associated with this edge
            short switch_id = -1; //The ID of the switch type this edge represents
        };

    private: //Data
        //Note: we use a plain array and use small types for sizes to save space vs std::vector
        //      (using std::vector's nearly doubles the size of the class)
        std::unique_ptr<t_rr_edge[]> edges_ = nullptr;
        uint16_t num_edges_ = 0;
        uint16_t edges_capacity_ = 0;
        uint8_t num_non_configurable_edges_ = 0;

        int8_t cost_index_ = -1;
        int16_t rc_index_ = -1;

        int16_t xlow_ = -1;
        int16_t ylow_ = -1;
        int16_t xhigh_ = -1;
        int16_t yhigh_ = -1;

        t_rr_type type_ = NUM_RR_TYPES;
        union {
            e_direction direction; //Valid only for CHANX/CHANY
            e_side side; //Valid only for IPINs/OPINs
        } dir_side_;

        union {
            int16_t pin_num;
            int16_t track_num;
            int16_t class_num;
        } ptc_;
        uint16_t fan_in_ = 0;
        uint16_t capacity_ = 0;
};


/*
 * Reistance/Capacitance data for an RR Nodes
 *
 * In practice many RR nodes have the same values, so they are fly-weighted
 * to keep t_rr_node small. Each RR node holds an rc_index which allows
 * retrieval of it's RC data.
 *
 * R:  Resistance to go through an RR node.  This is only metal
 *     resistance (end to end, so conservative) -- it doesn't include the
 *     switch that leads to another rr_node.
 * C:  Total capacitance of an RR node.  Includes metal capacitance, the
 *     input capacitance of all switches hanging off the node, the
 *     output capacitance of all switches to the node, and the connection
 *     box buffer capacitances hanging off it.
 */
struct t_rr_rc_data {
    t_rr_rc_data(float Rval, float Cval);

    float R;
    float C;
};

/*
 * Returns the index to a t_rr_rc_data matching the specified values.
 *
 * If an existing t_rr_rc_data matches the specified R/C it's index
 * is returned, otherwise the t_rr_rc_data is created.
 *
 * The returned indicies index into DeviceContext.rr_rc_data.
 */
short find_create_rr_rc_data(const float R, const float C);

#endif
