#include <cstdarg>
#include <cstdarg>
#include "vtr_util.h"
#include "rr_graph_error.h"

void rr_graph_throw(const char* filename, int line, const char* fmt, ...) {
    va_list va_args;

    va_start(va_args, fmt);

    auto msg = vtr::vstring_fmt(fmt, va_args);

    va_end(va_args);

    throw RRGraphError(msg, filename, line);
}

