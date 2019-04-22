
#include "vtr_log.h"
#include "vtr_util.h"

#include "clb2clb_directs.h"

/* Member functions of Clb2ClbDirectPort */
const t_type_descriptor* Clb2ClbDirectPort::get_clb_type() const { 
  return clb_type_;
}

int Clb2ClbDirectPort::get_pin_start_index() const {
  return pin_start_index_;
}

int Clb2ClbDirectPort::get_pin_end_index() const {
  return pin_end_index_;
}

void Clb2ClbDirectPort::set_clb_type(const t_type_descriptor* type) {
  clb_type_ = type;
  return;
}

void Clb2ClbDirectPort::set_pin_start_index(int pin_start_index) {
  pin_start_index_ = pin_start_index;
  return;
}

void Clb2ClbDirectPort::set_pin_end_index(int pin_end_index) { 
  pin_end_index_ = pin_end_index;
  return;
}

/* Create a fast look-up for the pins in the clb_type_
 * This will ease the searching for a pin index in a type
 */
void Clb2ClbDirectPort::create_type_pin_lookup() {

  int iport, ipin;
  int blk_pin_count, num_port_pins, num_ports;

  /* Clean the current look-up */
  type_pin_lookup_.clear();

  /* Allocate and load the indices. */
  num_ports = this->get_clb_type()->pb_type->num_ports;
  type_pin_lookup_.resize(num_ports);

  blk_pin_count = 0;
  for (iport = 0; iport < num_ports; ++iport) {
    num_port_pins = this->get_clb_type()->pb_type->ports[iport].num_pins;
    type_pin_lookup_[iport].resize(num_port_pins);
    for (ipin = 0; ipin < num_port_pins; ++ipin) {
      type_pin_lookup_[iport][ipin] = blk_pin_count;
    }
    blk_pin_count++;
  }

  return;
}


/***************************************************************************************
 * Xifan Tang: Adapt from the original version authored by 
  Y.G.THIEN
  30 AUG 2012

 * The following functions parses the direct connections' information obtained from    *
 * the arch file. Then, the functions map the block pins indices for all block types   *
 * to the corresponding idirect (the index of the direct connection as specified in    *
 * the arch file) and direct type (whether this pin is a SOURCE or a SINK for the      *
 * direct connection). If a pin is not part of any direct connections, the value       *
 * OPEN (-1) is stored in both entries.                                                *
 *                                                                                     *
 * The mapping arrays are freed by the caller. Currently, this mapping is only used to *
 * load placement macros in place_macro.c                                              *
 *                                                                                     *
 ***************************************************************************************/
void Clb2ClbDirectPort::parse(const std::string& src_string, 
                              const int line,
                              const DeviceTypes& device_types) {
  /* Parses out the pb_type_name and port_name from the direct passed in.   *
   * If the start_pin_index and end_pin_index is specified, parse them too. *
   * Return the values parsed by reference.                                 */

  /* Syntax delims */
  const std::string directs_delim(" ");
  const std::string type_pin_delim(".");
  const std::string pin_start_delim("[");
  const std::string pin_index_delim(":");
  const std::string pin_end_delim("]");

  /* Try to split with a space, to check there is multiple type+ports defined 
   */
  if (vtr::split(src_string, directs_delim).size() > 1) {
    VTR_LOG_ERROR("Only a single port pin range specification allowed for direct connect (was: '%s')", 
                  src_string);
    exit(1);
  }

  /* Split the source string first,
   * Find out the pb_type name (1st part) 
   * and port_name + pin_indices (2nd part) 
   */
  std::vector<std::string> split_string = vtr::split(src_string, type_pin_delim);

  /* First part should be pb_type */
  /* See if there is name match in types */
  this->set_clb_type(nullptr);
  for (size_t j = 0; j < device_types.get_num_types(); ++j) {
    if (0 == split_string[0].compare(device_types.get_types(j)->name)) {
      /* Reach here, it means we have found type_descriptor, fill the clb2clb_directs */
      this->set_clb_type(device_types.get_types(j));
      break;
    }
  }
  /* Error out if we fail to find any */
  if (nullptr == this->get_clb_type()) {
    VTR_LOG_ERROR("Unable to find a type_descriptor name by for clb2clb directs pin definition %s", 
                  split_string[0], src_string);
    exit(1);
  }

  /* Further parse the second part */
  std::vector<std::string> pin_string = vtr::split(split_string[1], pin_start_delim);
  /* Set a format error flag */
  bool format_err = false;

  /* Check if we have any pin index definition */
  if (1 == pin_string.size()) {
    /* Only port is defined, this indicates a full port is used */
    /* Set both starting and ending indices to be OPEN */
    this->set_pin_start_index(OPEN);
    this->set_pin_end_index(OPEN);
  } else if (2 == pin_string.size()) {
    /* Format check for [XX:XX]*/
    /* Error out if it is not started by '['  */
    if (pin_start_delim.back() != pin_string[1].front()) {
      format_err = true;
    }
    /* Remove the first '[' */
    pin_string[1].erase(0, 1);
    /* Error out if we can find any more '[' */
    if (std::string::npos == pin_string[1].find(pin_start_delim)) {
      format_err = true;
    }

    /* Error out if it is ended by ']'  */
    if (pin_end_delim.back() != pin_string[1].back()) {
      format_err = true;
    }
    /* Remove the last ']' */
    pin_string[1].pop_back();
    /* Error out if we can find any more ']' */
    if (std::string::npos == pin_string[1].find(pin_end_delim)) {
      format_err = true;
    }

    /* Further Split, find the delim ':' */
    std::vector<std::string> index_string = vtr::split(pin_string[1], pin_index_delim);
    /* If we can only find 1 string, it indicates a single pin */
    if (1 == index_string.size()) {
      this->set_pin_start_index(std::stoi(index_string[0]));
      this->set_pin_end_index(std::stoi(index_string[0]));
    } else if (2 == index_string.size()) {
    /* If we can find 2 string, it indicates a range of pins */
      this->set_pin_start_index(std::stoi(index_string[0]));
      this->set_pin_end_index(std::stoi(index_string[1]));
    } else {
      format_err = true;
    }
  } else { /* This is a broken string, error out */
    format_err = true;
  }
  
  /* Error out if format check reports so */
  if (true == format_err) { 
    VTR_LOG_ERROR("Invalid format of port definition %s! Expected port_name[pin_start_index:pin_end_index]", 
                  src_string);
    exit(1);
  }
 
  t_port* direct_port = nullptr;
  /* Now, deal with the port name and find it in types */
  for (size_t iport = 0; iport < this->get_clb_type()->pb_type->num_ports; ++iport) {
    if (0 == pin_string[0].compare(this->get_clb_type()->pb_type->ports[iport].name)) {
      /* We find the port ! */
      direct_port = &(this->get_clb_type()->pb_type->ports[iport]);
      break;
    }
  }
  /* Error out if we find nothing */
  if (nullptr == direct_port) { 
    VTR_LOG_ERROR("Failed to find a port %s in the defined complex blocks %s!", 
                  pin_string[0], this->get_clb_type()->name);
    exit(1);
  }

  /* Deal with pin indices when the full port is used for this direct */
  if (  (OPEN == this->get_pin_start_index()) 
     && (OPEN == this->get_pin_end_index()) ) {
    this->set_pin_start_index(0);
    this->set_pin_start_index(direct_port->num_pins - 1);
  }

  /* Check if the pin indices are in the range */
  if (  (0 > this->get_pin_start_index())
     || (direct_port->num_pins - 1 < this->get_pin_start_index()) ) {
    VTR_LOG_ERROR("Starting pin index of %s is out of the range of %s.port %s[%d:0]!", 
                  pin_string[0], this->get_clb_type()->name, direct_port->name, direct_port->num_pins - 1);
    exit(1);
  }

  if (  (0 > this->get_pin_end_index())
     || (direct_port->num_pins - 1 < this->get_pin_end_index()) ) {
    VTR_LOG_ERROR("Ending pin index of %s is out of the range of %s.port %s[%d:0]!", 
                  pin_string[0], this->get_clb_type()->name, direct_port->name, direct_port->num_pins - 1);
    exit(1);
  }

  /* Revert the pin indices if users define end_index < start_index */
  if (this->get_pin_end_index() < this->get_pin_start_index() ) {
    std::swap(this->pin_start_index_, this->pin_end_index_);
  }

  /* Update pin index, the pin index should be mapped to the global index in this pb_type */
  this->create_type_pin_lookup();

  /* Find the pin_start_index in the type port lists */
  int type_id = this->get_clb_type()->index;
  int port_id = direct_port->index;
  int pin_id = this->get_pin_start_index();
  this->set_pin_start_index(this->type_pin_lookup_[port_id][pin_id]);

  /* Find the pin_end_index in the type port lists */
  pin_id = this->get_pin_end_index();
  this->set_pin_end_index(this->type_pin_lookup_[port_id][pin_id]);

  //We must be careful to clean-up anything that we may have incidentally allocated.
  //Specifically, we can be called while generating the dummy architecture
  //for placer delay estimation.  Since the delay estimation occurs on a
  //'different' architecture it is almost certain that the f_blk_pin_from_port_pin allocated
  //by calling get_blk_pin_from_port_pin() will later be invalid.
  //We therefore must free it now.
  this->type_pin_lookup_.clear();

  return;
}

/* Access to internal data */
int Clb2ClbDirects::get_num_directs() const {
  return this->directs_.size();
}

/* validate if direct_id is in range */
bool Clb2ClbDirects::valid_direct_id(const size_t direct_id) const {
  return size_t(direct_id) < this->directs_.size() 
         && size_t(direct_id) < this->from_directs_.size()
         && size_t(direct_id) < this->to_directs_.size();
}

t_direct_inf Clb2ClbDirects::get_direct(const size_t direct_id) const {
  VTR_ASSERT(this->valid_direct_id(direct_id));
  return this->directs_[direct_id]; 
} 

Clb2ClbDirectPort Clb2ClbDirects::get_clb2clb_from_direct(const size_t direct_id) const {
  VTR_ASSERT(this->valid_direct_id(direct_id));
  return this->from_directs_[direct_id]; 
} 

Clb2ClbDirectPort Clb2ClbDirects::get_clb2clb_to_direct(const size_t direct_id) const {
  VTR_ASSERT(this->valid_direct_id(direct_id));
  return this->to_directs_[direct_id]; 
} 

/* Mutators */
void Clb2ClbDirects::set_directs(const size_t num_directs, const t_direct_inf* directs) {
  /* allocate vector */
  directs_.resize(num_directs);
  
  for (int idirect = 0; idirect < num_directs; ++idirect) {
    directs_[idirect] = directs[idirect]; 
  }

  return;
}

void Clb2ClbDirects::create_clb2clb_directs(const DeviceTypes& device_types,
                                            const short default_switch_id) {
  /* Allocate clb2clb_directs*/
  this->from_directs_.resize(this->get_num_directs());
  this->to_directs_.resize(this->get_num_directs());

  /* For each raw data, we parse the text and fill the clb2clb_directs */
  for (size_t idirect = 0; idirect < this->get_num_directs(); ++idirect) {
    /* Parse from_directs */
    this->from_directs_[idirect].parse(directs_[idirect].from_pin, directs_[idirect].line, 
                                       device_types);
    
    /* Parse to_directs */
    this->to_directs_[idirect].parse(directs_[idirect].to_pin, directs_[idirect].line,
                                     device_types);

    //Set the switch index
    if (directs_[idirect].switch_type > 0) {
      //Use the specified switch
      this->switch_ids_[idirect] = this->directs_[idirect].switch_type;
    } else {
      //Use the delayless switch by default
      this->switch_ids_[idirect] = default_switch_id;
    }
  }

  return;
}

/* Constuctor */
void Clb2ClbDirects::init(const int num_directs, const t_direct_inf* directs, 
                          const DeviceTypes& device_types,
                          const short default_switch_id) {
  this->set_directs(num_directs, directs);
  this->create_clb2clb_directs(device_types, default_switch_id);

  return;
}

