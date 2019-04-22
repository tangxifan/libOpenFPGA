#include "device_types.h"

bool DeviceTypes::is_input_type(t_type_ptr type) const {
  return this->input_types_.count(type);
}

bool DeviceTypes::is_output_type(t_type_ptr type) const {
  return this->output_types_.count(type);
}

bool DeviceTypes::is_io_type(t_type_ptr type) const {
  return this->is_input_type(type)
      || this->is_output_type(type);
}

bool DeviceTypes::is_empty_type(t_type_ptr type) const {
  return type == EMPTY_TYPE_;
}

/* Accessors */
int DeviceTypes::get_num_types() const {
  return this->types_.size();
}

const t_type_descriptor* DeviceTypes::get_types(int itype) const {
  return &(types_[itype]);
}


/* Mutators */
void DeviceTypes::set_empty_type(t_type_ptr empty_type) {
  this->EMPTY_TYPE_ = empty_type;
  
  return;
}

void DeviceTypes::set_types(const int num_types, const t_type_descriptor* types) {
  /* Ensure a clean start */
  this->types_.clear();
 
  this->types_.resize(num_types); 
   
  for (int itype = 0; itype < num_types; itype++) {
    this->types_[itype] = types[itype];
  }

  return;
} 
