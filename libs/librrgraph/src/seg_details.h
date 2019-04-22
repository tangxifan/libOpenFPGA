#ifndef SEG_DETAILS_H
#define SEG_DETAILS_H

#include <memory>
#include "rr_graph_node_types.h"
#include "chan_width.h"
#include "device_types.h"

constexpr std::array<const char*, NUM_DIRECTIONS> SEG_DIRECTION_STRING = { 
  {"INC_DIRECTION", "DEC_DIRECTION", "BI_DIRECTION", "NO_DIRECTION"} 
};

enum e_seg_details_type {
	SEG_DETAILS_X, SEG_DETAILS_Y
};

/* Lists detailed information about segmentation.  [0 .. W-1].              *
 * length:  length of segment.                                              *
 * start:  index at which a segment starts in channel 0.                    *
 * longline:  true if this segment spans the entire channel.                *
 * sb:  [0..length]:  true for every channel intersection, relative to the  *
 *      segment start, at which there is a switch box.                      *
 * cb:  [0..length-1]:  true for every logic block along the segment at     *
 *      which there is a connection box.                                    *
 * arch_wire_switch: Index of the switch type that connects other wires     *
 *                   *to* this segment. Note that this index is in relation *
 *                   to the switches from the architecture file, not the    *
 *                   expanded list of switches that is built at the end of  *
 *                   build_rr_graph.                                        *
 * arch_opin_switch: Index of the switch type that connects output pins     *
 *                   (OPINs) *to* this segment. Note that this index is in  *
 *                   relation to the switches from the architecture file,   *
 *                   not the expanded list of switches that is is built     *
 *                   at the end of build_rr_graph                           *
 * Cmetal: Capacitance of a routing track, per unit logic block length.     *
 * Rmetal: Resistance of a routing track, per unit logic block length.      *
 * direction: The direction of a routing track.                             *
 * index: index of the segment type used for this track.                    *
 * type_name_ptr: pointer to name of the segment type this track belongs    *
 *                to. points to the appropriate name in s_segment_inf       */
struct t_seg_details {
  int length = 0;
  int start = 0;
  bool longline = 0;
  std::unique_ptr<bool[]> sb;
  std::unique_ptr<bool[]> cb;
  short arch_wire_switch = 0;
  short arch_opin_switch = 0;
  float Rmetal = 0;
  float Cmetal = 0;
  bool twisted = 0;
  enum e_seg_direction direction = NO_DIRECTION;
  int group_start = 0;
  int group_size = 0;
  int seg_start = 0;
  int seg_end = 0;
  int index = 0;
  float Cmetal_per_m = 0; /* Used for power */
  std::string type_name;
};

class t_chan_seg_details {
    public:
        t_chan_seg_details() = default;
        t_chan_seg_details(const t_seg_details* init_seg_details)
            : length_(init_seg_details->length)
            , seg_detail_(init_seg_details) {}

    public:
        int length() const { return length_; }
        int seg_start() const { return seg_start_; }
        int seg_end() const { return seg_end_; }

        int start() const { return seg_detail_->start; }
        bool longline() const { return seg_detail_->longline; }

        int group_start() const { return seg_detail_->group_start; }
        int group_size() const { return seg_detail_->group_size; }

        bool cb(int pos) const { return seg_detail_->cb[pos]; }
        bool sb(int pos) const { return seg_detail_->sb[pos]; }

        float Rmetal() const { return seg_detail_->Rmetal; }
        float Cmetal() const { return seg_detail_->Cmetal; }
        float Cmetal_per_m() const { return seg_detail_->Cmetal_per_m; }

        short arch_wire_switch() const { return seg_detail_->arch_wire_switch; }
        short arch_opin_switch() const { return seg_detail_->arch_opin_switch; }

        e_seg_direction direction() const { return seg_detail_->direction; }

        int index() const { return seg_detail_->index; }

        std::string type_name() const { return seg_detail_->type_name; }

    public: //Modifiers
        void set_length(int new_len) { length_ = new_len; }
        void set_seg_start(int new_start) { seg_start_ = new_start; }
        void set_seg_end(int new_end) { seg_end_ = new_end; }

    private:
        //The only unique information about a channel segment is it's start/end
        //and length.  All other information is shared accross segment types,
        //so we use a flyweight to the t_seg_details which defines that info.
        //
        //To preserve the illusion of uniqueness we wrap all t_seg_details members
        //so it appears transparent -- client code of this class doesn't need to
        //know about t_seg_details.
        int length_ = -1;
        int seg_start_ = -1;
        int seg_end_ = -1;
        const t_seg_details* seg_detail_ = nullptr;
};

/* Defines a 2-D array of t_seg_details data structures (one per channel)   */
typedef vtr::NdMatrix<t_chan_seg_details,3> t_chan_details;

/* Subroutines */
/* FIXME: the functions should be in class */
int *get_seg_track_counts(
        const int num_sets,
        const std::vector<t_segment_inf>& segment_inf,
        const bool use_full_seg_groups);

t_seg_details* alloc_and_load_global_route_seg_details(const int global_route_switch,
                                                       int* num_seg_details);

t_seg_details *alloc_and_load_seg_details(
        int *max_chan_width, const int max_len,
        const std::vector<t_segment_inf>& segment_inf,
        const bool use_full_seg_groups, const bool is_global_graph,
        const enum e_directionality directionality,
        int * num_seg_details);

void alloc_and_load_chan_details(
        const DeviceTypes& device_types,
        const DeviceGrid& grid,
        const t_chan_width* nodes_per_chan,
        const bool trim_empty_channels,
        const bool trim_obs_channels,
        const int num_seg_details,
        const t_seg_details* seg_details,
        t_chan_details& chan_details_x,
        t_chan_details& chan_details_y);


#endif
