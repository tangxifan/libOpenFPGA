/* IMPORTANT:
 * The following preprocessing flags are added to 
 * avoid compilation error when this headers are included in more than 1 times 
 */
#ifndef CLB2CLB_DIRECTS_H
#define CLB2CLB_DIRECTS_H

#include <string>
#include <vector>

#include "arch_types.h"
#include "device_types.h"

/*
 * A class for clb_to_clb_directs
 * which defines direction interconnections between CLBs
 * This structure translates from architecture XML file to a detailed description.
 * Its information is frequently used by VPR packing, placement and routing engine.
 */

/* This class should be only an internal class of Clb2ClbDirects */
class Clb2ClbDirectPort {
  public: /* Constructor */
    void parse(const std::string& src_string, const int line,
               const DeviceTypes& device_types);
  public: /* Access */
    const t_type_descriptor* get_clb_type() const;
    int get_pin_start_index() const;
    int get_pin_end_index() const;
  private: /* Mutators */
    void set_clb_type(const t_type_descriptor* type);
    void set_pin_start_index(int start_index);
    void set_pin_end_index(int end_index);
    void create_type_pin_lookup();
  private: /* Internal data */
    const t_type_descriptor* clb_type_;
    int pin_start_index_;
    int pin_end_index_;
    /* look-up for pin-index in the type */
    std::vector<std::vector<int>> type_pin_lookup_;
};

class Clb2ClbDirects {
  public: /* Constructors */
    void init(const int num_directs, const t_direct_inf* directs, 
              const DeviceTypes& device_types,
              const short default_switch_id);

  public: /* Access to internal data */
    int get_num_directs() const;
    bool valid_direct_id(const size_t direct_id) const; /* validate if direct_id is in range */
    t_direct_inf get_direct(const size_t direct_id) const; 
    Clb2ClbDirectPort get_clb2clb_from_direct(const size_t direct_id) const; 
    Clb2ClbDirectPort get_clb2clb_to_direct(const size_t direct_id) const; 
    
  private: /* Mutators: data allocation and loading */
    void set_directs(const size_t num_directs, const t_direct_inf* directs);
    void create_clb2clb_directs(const DeviceTypes& device_types,
                                const short default_switch_id); /* Convert directs to clb2clb_directs*/

  private: /* Internal Data */
    /* Raw Data parsed from architecture XML */
    std::vector<t_direct_inf> directs_;
 
    /* Data translated from architecture description
     * showing detailed information about inter-clb direct connections  
     */
    std::vector<Clb2ClbDirectPort> from_directs_;
    std::vector<Clb2ClbDirectPort> to_directs_;
    /* Switch info 
     * The switch type used by this direct connection
     */
    std::vector<int> switch_ids_;
};

#endif
