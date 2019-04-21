/* IMPORTANT:
 * The following preprocessing flags are added to 
 * avoid compilation error when this headers are included in more than 1 times 
 */
#ifndef CLB_TO_CLB_DIRECTS_H
#define CLB_TO_CLB_DIRECTS_H

#include "arch_types.h"

/*
 * A class for clb_to_clb_directs
 * which defines direction interconnections between CLBs
 * This structure translates from architecture XML file to a detailed description.
 * Its information is frequently used by VPR packing, placement and routing engine.
 */
struct t_clb_to_clb_directs {
    t_type_descriptor* from_clb_type_;
    int from_clb_pin_start_index_;
    int from_clb_pin_end_index_;
    t_type_descriptor* to_clb_type_;
    int to_clb_pin_start_index_;
    int to_clb_pin_end_index_;
    int switch_index_; //The switch type used by this direct connection
};

class Clb2ClbDirects {
  public: /* Constructors */
    init(const size_t num_directs, const t_direct_inf* directs);

  public: /* Access to internal data */
    size_t num_directs() const; 
    bool valid_direct_id(size_t direct_id) const; /* validate if direct_id is in range */
    t_direct_inf get_direct(size_t direct_id) const; 
    t_direct_inf& get_direct_ptr(size_t direct_id) const; 
    t_clb_to_clb_directs get_clb2clb_direct(size_t direct_id) const; 
    t_clb_to_clb_directs* get_clb2clb_direct_ptr(size_t direct_id) const; 
    
  private: /* Mutators: data allocation and loading */
    void set_num_directs(size_t num_directs);
    void parse_direct_pin_name(char * src_string, int line, int * start_pin_index,
	                           int * end_pin_index, char * pb_type_name, char * port_name);
    void create_clb2clb_directs(); /* Convert directs to clb2clb_directs*/


  private: /* Internal Data */
    /* Raw Data parsed from architecture XML */
    size_t num_directs_;
    std::vector<t_direct_inf> directs_;
 
    /* Data translated from architecture description
     * showing detailed information about inter-clb direct connections  
     */
    std::vector<t_clb_to_clb_directs> clb2clb_directs_;
};

