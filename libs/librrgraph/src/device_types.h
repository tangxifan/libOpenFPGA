#ifndef DEVICE_TYPES_H
#define DEVICE_TYPES_H

#include <vector>
#include <set>

#include "arch_types.h"

/* A data structure containing type_descriptors
 * and lists to categorize types 
 */
class DeviceTypes {
  public: /* Constructors */
    /* TODO: add creators */
  public: /* Inquiry functions */
    bool is_input_type(t_type_ptr type) const;
    bool is_output_type(t_type_ptr type) const;
    bool is_io_type(t_type_ptr type) const;
    bool is_empty_type(t_type_ptr type) const;
  public: /* Accessors */
    int get_num_types() const;
    const t_type_descriptor* get_types(int itype) const;
  private: /* Mutators */
    void set_types(const int num_types, const t_type_descriptor* types); 
    void set_empty_type(t_type_ptr empty_type); 
    /* TODO: add set creators */
  private: /* internal data */
    std::vector<t_type_descriptor> types_;

    /* Special pointers to identify special blocks on an FPGA: I/Os, unused, and default */
    std::set<t_type_ptr> input_types_;
    std::set<t_type_ptr> output_types_;
    t_type_ptr EMPTY_TYPE_;
};

#endif 

