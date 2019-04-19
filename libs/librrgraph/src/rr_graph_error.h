#ifndef RRGRAPH_ERROR_H
#define RRGRAPH_ERROR_H

#include "vtr_error.h"
#include <cstdarg>

#define RR_GRAPH_ERROR "librrgraph"

void rr_graph_throw(const char* filename, int line, const char* fmt, ...);

class RRGraphError : public vtr::VtrError {
    public:
        RRGraphError(std::string msg="", std::string new_filename="", size_t new_linenumber=-1)
            : vtr::VtrError(msg, new_filename, new_linenumber){}
};

/*
 * Macro wrapper around rr_graph_throw() which automatically
 * specifies file and line number of call site.
 */
#define RR_GRAPH_THROW(...) do { \
        rr_graph_throw(__FILE__, __LINE__, __VA_ARGS__); \
    } while(false)

#endif
