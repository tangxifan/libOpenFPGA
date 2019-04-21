#include "clb_to_clb_directs.h"

/* Access to internal data */
size_t Clb2ClbDirects::num_directs() const {
  return this->num_directs_;
}

/* validate if direct_id is in range */
bool Clb2ClbDirects::valid_direct_id(size_t direct_id) const {
  return size_t(direct_id) < this->directs_.size() && size_t(direct_id) < this->clb2clb_directs_.size();
}

t_direct_inf Clb2ClbDirects::get_direct(size_t direct_id) const {
  VTR_ASSERT(this->valid_direct_id(direct_id));
  return this->directs_.[direct_id]; 
} 

t_direct_inf* Clb2ClbDirects::get_direct_ptr(size_t direct_id) const {
  VTR_ASSERT(this->valid_direct_id(direct_id));
  return this->directs_.data(direct_id); 
} 

t_clb_to_clb_directs Clb2ClbDirects::get_clb2clb_direct(size_t direct_id) const {
  VTR_ASSERT(this->valid_direct_id(direct_id));
  return this->clb2clb_directs_[direct_id]; 
} 

t_clb_to_clb_directs* Clb2ClbDirects::get_clb2clb_direct_ptr(size_t direct_id) const {
  VTR_ASSERT(this->valid_direct_id(direct_id));
  return this->clb2clb_directs_.data(direct_id); 
}

/* Mutators */
void Clb2ClbDirects::set_num_directs(size_t num_directs) {
  this->num_directs_ = num_directs;
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
void parse_direct_pin_name() {
  /* Parses out the pb_type_name and port_name from the direct passed in.   *
   * If the start_pin_index and end_pin_index is specified, parse them too. *
   * Return the values parsed by reference.                                 */

  char source_string[MAX_STRING_LEN+1];
  char * find_format = nullptr;
  int ichar, match_count;

  if (vtr::split(src_string).size() > 1) {
      vpr_throw(VPR_ERROR_ARCH, __FILE__, __LINE__,
                "Only a single port pin range specification allowed for direct connect (was: '%s')", 
                src_string);
    }

  // parse out the pb_type and port name, possibly pin_indices
  find_format = strstr(src_string,"[");
  if (find_format == nullptr) {
    /* Format "pb_type_name.port_name" */
    *start_pin_index = *end_pin_index = -1;

    if(strlen(src_string) + 1 <= MAX_STRING_LEN + 1) {
            strcpy (source_string, src_string);
        } else {
            vpr_throw(VPR_ERROR_ARCH, __FILE__, __LINE__,
                      "Pin name exceeded buffer size of %zu characters", MAX_STRING_LEN + 1);

        }
    for (ichar = 0; ichar < (int)(strlen(source_string)); ichar++) {
      if (source_string[ichar] == '.')
        source_string[ichar] = ' ';
    }

    match_count = sscanf(source_string, "%s %s", pb_type_name, port_name);
    if (match_count != 2){
      VTR_LOG_ERROR(
          "[LINE %d] Invalid pin - %s, name should be in the format "
          "\"pb_type_name\".\"port_name\" or \"pb_type_name\".\"port_name[end_pin_index:start_pin_index]\". "
          "The end_pin_index and start_pin_index can be the same.\n",
          line, src_string);
      exit(1);
    }
  } else {
    /* Format "pb_type_name.port_name[end_pin_index:start_pin_index]" */
    strcpy (source_string, src_string);
    for (ichar = 0; ichar < (int)(strlen(source_string)); ichar++) {
            //Need white space between the components when using %s with
            //sscanf
      if (source_string[ichar] == '.')
        source_string[ichar] = ' ';
      if (source_string[ichar] == '[')
        source_string[ichar] = ' ';
    }

    match_count = sscanf(source_string, "%s %s %d:%d]",
                pb_type_name, port_name,
                end_pin_index, start_pin_index);
    if (match_count != 4){
      VTR_LOG_ERROR(
          "[LINE %d] Invalid pin - %s, name should be in the format "
          "\"pb_type_name\".\"port_name\" or \"pb_type_name\".\"port_name[end_pin_index:start_pin_index]\". "
          "The end_pin_index and start_pin_index can be the same.\n",
          line, src_string);
      exit(1);
    }
    if (*end_pin_index < 0 || *start_pin_index < 0) {
      VTR_LOG_ERROR(
          "[LINE %d] Invalid pin - %s, the pin_index in "
          "[end_pin_index:start_pin_index] should not be a negative value.\n",
          line, src_string);
      exit(1);
    }
    if ( *end_pin_index < *start_pin_index) {
      VTR_LOG_ERROR(
          "[LINE %d] Invalid from_pin - %s, the end_pin_index in "
          "[end_pin_index:start_pin_index] should not be less than start_pin_index.\n",
          line, src_string);
      exit(1);
    }
  }

  return;
}

void Clb2ClbDirects::create_clb2clb_directs(size_t default_switch_id) {
  int i, j;
  t_clb_to_clb_directs *clb_to_clb_directs;
  char *pb_type_name, *port_name;
  int start_pin_index, end_pin_index;
  t_pb_type *pb_type;

  auto& device_ctx = g_vpr_ctx.device();

  clb_to_clb_directs = (t_clb_to_clb_directs*) vtr::calloc(num_directs, sizeof (t_clb_to_clb_directs));

  pb_type_name = nullptr;
  port_name = nullptr;

  for (i = 0; i < num_directs; i++) {
      pb_type_name = (char*) vtr::malloc((strlen(directs[i].from_pin) + strlen(directs[i].to_pin)) * sizeof (char));
      port_name = (char*) vtr::malloc((strlen(directs[i].from_pin) + strlen(directs[i].to_pin)) * sizeof (char));

      // Load from pins
      // Parse out the pb_type name, port name, and pin range
      parse_direct_pin_name(directs[i].from_pin, directs[i].line, &start_pin_index, &end_pin_index, pb_type_name, port_name);

      // Figure out which type, port, and pin is used
      for (j = 0; j < device_ctx.num_block_types; j++) {
          if (strcmp(device_ctx.block_types[j].name, pb_type_name) == 0) {
              break;
          }
      }
      if (j >= device_ctx.num_block_types) {
          vpr_throw(VPR_ERROR_ARCH, get_arch_file_name(), directs[i].line, "Unable to find block %s.\n", pb_type_name);
      }
      clb_to_clb_directs[i].from_clb_type = &device_ctx.block_types[j];
      pb_type = clb_to_clb_directs[i].from_clb_type->pb_type;

      for (j = 0; j < pb_type->num_ports; j++) {
          if (strcmp(pb_type->ports[j].name, port_name) == 0) {
              break;
          }
      }
      if (j >= pb_type->num_ports) {
          vpr_throw(VPR_ERROR_ARCH, get_arch_file_name(), directs[i].line, "Unable to find port %s (on block %s).\n", port_name, pb_type_name);
      }

      if (start_pin_index == OPEN) {
          VTR_ASSERT(start_pin_index == end_pin_index);
          start_pin_index = 0;
          end_pin_index = pb_type->ports[j].num_pins - 1;
      }
      get_blk_pin_from_port_pin(clb_to_clb_directs[i].from_clb_type->index, j, start_pin_index, &clb_to_clb_directs[i].from_clb_pin_start_index);
      get_blk_pin_from_port_pin(clb_to_clb_directs[i].from_clb_type->index, j, end_pin_index, &clb_to_clb_directs[i].from_clb_pin_end_index);

      // Load to pins
      // Parse out the pb_type name, port name, and pin range
      parse_direct_pin_name(directs[i].to_pin, directs[i].line, &start_pin_index, &end_pin_index, pb_type_name, port_name);

      // Figure out which type, port, and pin is used
      for (j = 0; j < device_ctx.num_block_types; j++) {
          if (strcmp(device_ctx.block_types[j].name, pb_type_name) == 0) {
              break;
          }
      }
      if (j >= device_ctx.num_block_types) {
          vpr_throw(VPR_ERROR_ARCH, get_arch_file_name(), directs[i].line, "Unable to find block %s.\n", pb_type_name);
      }
      clb_to_clb_directs[i].to_clb_type = &device_ctx.block_types[j];
      pb_type = clb_to_clb_directs[i].to_clb_type->pb_type;

      for (j = 0; j < pb_type->num_ports; j++) {
          if (strcmp(pb_type->ports[j].name, port_name) == 0) {
              break;
          }
      }
      if (j >= pb_type->num_ports) {
          vpr_throw(VPR_ERROR_ARCH, get_arch_file_name(), directs[i].line, "Unable to find port %s (on block %s).\n", port_name, pb_type_name);
      }

      if (start_pin_index == OPEN) {
          VTR_ASSERT(start_pin_index == end_pin_index);
          start_pin_index = 0;
          end_pin_index = pb_type->ports[j].num_pins - 1;
      }

      get_blk_pin_from_port_pin(clb_to_clb_directs[i].to_clb_type->index, j, start_pin_index, &clb_to_clb_directs[i].to_clb_pin_start_index);
      get_blk_pin_from_port_pin(clb_to_clb_directs[i].to_clb_type->index, j, end_pin_index, &clb_to_clb_directs[i].to_clb_pin_end_index);

      if (abs(clb_to_clb_directs[i].from_clb_pin_start_index - clb_to_clb_directs[i].from_clb_pin_end_index) != abs(clb_to_clb_directs[i].to_clb_pin_start_index - clb_to_clb_directs[i].to_clb_pin_end_index)) {
          vpr_throw(VPR_ERROR_ARCH, get_arch_file_name(), directs[i].line,
                  "Range mismatch from %s to %s.\n", directs[i].from_pin, directs[i].to_pin);
      }

      //Set the switch index
      if (directs[i].switch_type > 0) {
          //Use the specified switch
          clb_to_clb_directs[i].switch_index = directs[i].switch_type;
      } else {
          //Use the delayless switch by default
          clb_to_clb_directs[i].switch_index = delayless_switch;

      }
      free(pb_type_name);
      free(port_name);

      //We must be careful to clean-up anything that we may have incidentally allocated.
      //Specifically, we can be called while generating the dummy architecture
      //for placer delay estimation.  Since the delay estimation occurs on a
      //'different' architecture it is almost certain that the f_blk_pin_from_port_pin allocated
      //by calling get_blk_pin_from_port_pin() will later be invalid.
      //We therefore must free it now.
      free_blk_pin_from_port_pin();

  }
  return;
}
