/*
 Test librrgraph, try reading an architecture and generate rr_graph 

 Date: April 15, 2019
 Author: Xifan Tang
 */

#include <stdio.h>
#include <stdlib.h>

#include "vtr_error.h"
#include "vtr_memory.h"
#include "read_xml_arch_file.h"
#include "echo_arch.h"
#include "rr_graph.h"

void print_help() {
  printf("\n---------------------------------------------------------------------------------------\n");
  printf("read_arch - Read a VPR architecture file and output internal data structures\n");
  printf("\n");
  printf("Usage: read_arch <arch_file.xml> <timing_driven (0|1)> <output_file>\n");
  printf("\n");
  printf("  ex: read_arch k4_n10.xml 1 arch_data.out\n");
  printf("      Read timing-driven architecture k4_n10.xml and output the results to arch_data.out\n");
  printf("\n---------------------------------------------------------------------------------------\n");
}

int main(int argc, char **argv) {

  try {
    /* Read options */
    if (2 != argc - 1) { /* First argv is the name of program */
      printf("Error: Unexpected # of arguments. Expected 2 found %d arguements\n",
             argc);
      print_help();
      return 1;
    }

    printf("------------------------------------------------------------------------------\n");
    printf("- Read architecture file and print library data structures into an output file\n");
    printf("------------------------------------------------------------------------------\n\n");

    printf("Inputs: \n"
           "architecture %s \n"
           "timing_driven %d \n"
           "output file %s\n", argv[1], atoi(argv[2]), argv[3]);
    printf("Reading in architecture\n");

    /* Read architecture XML and build_rr_graph options */
    t_arch* arch = (t_arch*) vtr::calloc(1, sizeof(t_arch));
    t_type_descriptor* types = NULL;
    int numTypes;

    XmlReadArch(argv[1], atoi(argv[2]), arch, &types, &numTypes);

    printf("Printing Results\n");

    EchoArch(argv[3], types, numTypes, arch);

    /* Build the grid */

    /* Build the chan_width??? */

    /* Create the options for rr_graph builders */

    /* Create a rr_graph object and perform testing for major functions */
    RRGraph global_rr_graph;

    /* 
    rr_graph.build_detail_rr_graph();
    */
    /* Optional: dump the built rr_graph  */
    const char* outfile = "rr_graph_out.xml";
    global_rr_graph.dump_rr_graph_to_file(outfile);

    /* Deconstruct and free */
    free(arch);
  } catch (vtr::VtrError& vtr_error) {
    printf("Failed to build rr_graph from architecture %s: %s\n", argv[1], vtr_error.what());
    return 1;
  }

  printf("Done\n");

  return 0;
}

