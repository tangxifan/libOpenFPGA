#include <algorithm>

#include "seg_details.h"
#include "vtr_memory.h"
#include "rr_graph_error.h"


t_seg_details* alloc_and_load_global_route_seg_details(const int global_route_switch,
                                                       int* num_seg_details) {
  t_seg_details* seg_details = new t_seg_details[1];

  seg_details->index = 0;
  seg_details->length = 1;
  seg_details->arch_wire_switch = global_route_switch;
  seg_details->arch_opin_switch = global_route_switch;
  seg_details->longline = false;
  seg_details->direction = BI_DIRECTION;
  seg_details->Cmetal = 0.0;
  seg_details->Rmetal = 0.0;
  seg_details->start = 1;
  seg_details->cb = std::make_unique<bool[]>(1);
  seg_details->cb[0] = true;
  seg_details->sb = std::make_unique<bool[]>(2);
  seg_details->sb[0] = true;
  seg_details->sb[1] = true;
  seg_details->group_size = 1;
  seg_details->group_start = 0;
  seg_details->seg_start = -1;
  seg_details->seg_end = -1;

  if (num_seg_details) {
      *num_seg_details = 1;
  }
  return seg_details;
}

/* Returns the segment number at which the segment this track lies on        *
 * started.                                                                  */
int get_seg_start(
        const t_chan_seg_details * seg_details, const int itrack,
        const int chan_num, const int seg_num) {

    int seg_start = 0;
    if (seg_details[itrack].seg_start() >= 0) {

        seg_start = seg_details[itrack].seg_start();

    } else {

        seg_start = 1;
        if (false == seg_details[itrack].longline()) {

            int length = seg_details[itrack].length();
            int start = seg_details[itrack].start();

            /* Start is guaranteed to be between 1 and length.  Hence adding length to *
             * the quantity in brackets below guarantees it will be nonnegative.       */

            VTR_ASSERT(start > 0);
            VTR_ASSERT(start <= length);

            /* NOTE: Start points are staggered between different channels.
             * The start point must stagger backwards as chan_num increases.
             * Unidirectional routing expects this to allow the N-to-N
             * assumption to be made with respect to ending wires in the core. */
            seg_start = seg_num - (seg_num + length + chan_num - start) % length;
            if (seg_start < 1) {
                seg_start = 1;
            }
        }
    }
    return seg_start;
}

int get_seg_end(const t_chan_seg_details * seg_details, const int itrack, const int istart,
        const int chan_num, const int seg_max) {


    if (seg_details[itrack].longline()) {
        return seg_max;
    }

    if (seg_details[itrack].seg_end() >= 0) {
        return seg_details[itrack].seg_end();
    }

    int len = seg_details[itrack].length();
    int ofs = seg_details[itrack].start();

    /* Normal endpoint */
    int seg_end = istart + len - 1;

    /* If start is against edge it may have been clipped */
    if (1 == istart) {
        /* If the (staggered) startpoint of first full wire wasn't
         * also 1, we must be the clipped wire */
        int first_full = (len - (chan_num % len) + ofs - 1) % len + 1;
        if (first_full > 1) {
            /* then we stop just before the first full seg */
            seg_end = first_full - 1;
        }
    }

    /* Clip against far edge */
    if (seg_end > seg_max) {
        seg_end = seg_max;
    }

    return seg_end;
}


/* This assigns tracks (individually or pairs) to segment types.
 * It tries to match requested ratio. If use_full_seg_groups is
 * true, then segments are assigned only in multiples of their
 * length. This is primarily used for making a tileable unidir
 * layout. The effect of using this is that the number of tracks
 * requested will not always be met and the result will sometimes
 * be over and sometimes under.
 * The pattern when using use_full_seg_groups is to keep adding
 * one group of the track type that wants the largest number of
 * groups of tracks. Each time a group is assigned, the types
 * demand is reduced by 1 unit. The process stops when we are
 * no longer less than the requested number of tracks. As a final
 * step, if we were closer to target before last more, undo it
 * and end up with a result that uses fewer tracks than given. */
int *get_seg_track_counts(
        const int num_sets,
        const std::vector<t_segment_inf>& segment_inf,
        const bool use_full_seg_groups) {

    int *result;
    double *demand;
    int imax, freq_sum, assigned, size;
    double scale, max, reduce;

    result = (int *) vtr::malloc(sizeof (int) * segment_inf.size());
    demand = (double *) vtr::malloc(sizeof (double) * segment_inf.size());

    /* Scale factor so we can divide by any length
     * and still use integers */
    scale = 1;
    freq_sum = 0;
    for (size_t i = 0; i < segment_inf.size(); ++i) {
        scale *= segment_inf[i].length;
        freq_sum += segment_inf[i].frequency;
    }
    reduce = scale * freq_sum;

    /* Init assignments to 0 and set the demand values */
    for (size_t i = 0; i < segment_inf.size(); ++i) {
        result[i] = 0;
        demand[i] = scale * num_sets * segment_inf[i].frequency;
        if (use_full_seg_groups) {
            demand[i] /= segment_inf[i].length;
        }
    }

    /* Keep assigning tracks until we use them up */
    assigned = 0;
    size = 0;
    imax = 0;
    while (assigned < num_sets) {
        /* Find current maximum demand */
        max = 0;
        for (size_t i = 0; i < segment_inf.size(); ++i) {
            if (demand[i] > max) {
                imax = i;
                max = demand[i];
            }
        }

        /* Assign tracks to the type and reduce the types demand */
        size = (use_full_seg_groups ? segment_inf[imax].length : 1);
        demand[imax] -= reduce;
        result[imax] += size;
        assigned += size;
    }

    /* Undo last assignment if we were closer to goal without it */
    if ((assigned - num_sets) > (size / 2)) {
        result[imax] -= size;
    }

    /* Free temps */
    if (demand) {
        vtr::free(demand);
        demand = nullptr;
    }

    /* This must be freed by caller */
    return result;
}

t_seg_details *alloc_and_load_seg_details(
        int *max_chan_width, const int max_len,
        const std::vector<t_segment_inf>& segment_inf,
        const bool use_full_seg_groups, const bool is_global_graph,
        const enum e_directionality directionality,
        int * num_seg_details) {

    /* Allocates and loads the seg_details data structure.  Max_len gives the   *
     * maximum length of a segment (dimension of array).  The code below tries  *
     * to:                                                                      *
     * (1) stagger the start points of segments of the same type evenly;        *
     * (2) spread out the limited number of connection boxes or switch boxes    *
     *     evenly along the length of a segment, starting at the segment ends;  *
     * (3) stagger the connection and switch boxes on different long lines,     *
     *     as they will not be staggered by different segment start points.     */

    int cur_track, ntracks, itrack, length, j, index;
    int arch_wire_switch, arch_opin_switch, fac, num_sets, tmp;
    int group_start, first_track;
    int *sets_per_seg_type = nullptr;
    t_seg_details *seg_details = nullptr;
    bool longline;

    /* Unidir tracks are assigned in pairs, and bidir tracks individually */
    if (directionality == BI_DIRECTIONAL) {
        fac = 1;
    } else {
        VTR_ASSERT(directionality == UNI_DIRECTIONAL);
        fac = 2;
    }

    if (*max_chan_width % fac != 0) {
        RR_GRAPH_THROW("Routing channel width must be divisible by %d (channel width was %d)", fac, *max_chan_width);
    }

    /* Map segment type fractions and groupings to counts of tracks */
    sets_per_seg_type = get_seg_track_counts((*max_chan_width / fac),
            segment_inf, use_full_seg_groups);

    /* Count the number tracks actually assigned. */
    tmp = 0;
    for (size_t i = 0; i < segment_inf.size(); ++i) {
        tmp += sets_per_seg_type[i] * fac;
    }
    VTR_ASSERT(use_full_seg_groups || (tmp == *max_chan_width));
    *max_chan_width = tmp;

    seg_details = new t_seg_details[*max_chan_width];

    /* Setup the seg_details data */
    cur_track = 0;
    for (size_t i = 0; i < segment_inf.size(); ++i) {
        first_track = cur_track;

        num_sets = sets_per_seg_type[i];
        ntracks = fac * num_sets;
        if (ntracks < 1) {
            continue;
        }
        /* Avoid divide by 0 if ntracks */
        longline = segment_inf[i].longline;
        length = segment_inf[i].length;
        if (longline) {
            length = max_len;
        }

        arch_wire_switch = segment_inf[i].arch_wire_switch;
        arch_opin_switch = segment_inf[i].arch_opin_switch;
        VTR_ASSERT((arch_wire_switch == arch_opin_switch) || (directionality != UNI_DIRECTIONAL));

        /* Set up the tracks of same type */
        group_start = 0;
        for (itrack = 0; itrack < ntracks; itrack++) {

            /* set the name of the segment type this track belongs to */
            seg_details[cur_track].type_name = segment_inf[i].name;

            /* Remember the start track of the current wire group */
            if ((itrack / fac) % length == 0 && (itrack % fac) == 0) {
                group_start = cur_track;
            }

            seg_details[cur_track].length = length;
            seg_details[cur_track].longline = longline;

            /* Stagger the start points in for each track set. The
             * pin mappings should be aware of this when chosing an
             * intelligent way of connecting pins and tracks.
             * cur_track is used as an offset so that extra tracks
             * from different segment types are hopefully better
             * balanced. */
            seg_details[cur_track].start = (cur_track / fac) % length + 1;

            /* These properties are used for vpr_to_phy_track to determine
             * * twisting of wires. */
            seg_details[cur_track].group_start = group_start;
            seg_details[cur_track].group_size =
                    std::min(ntracks + first_track - group_start, length * fac);
            VTR_ASSERT(0 == seg_details[cur_track].group_size % fac);
            if (0 == seg_details[cur_track].group_size) {
                seg_details[cur_track].group_size = length * fac;
            }

            seg_details[cur_track].seg_start = -1;
            seg_details[cur_track].seg_end = -1;

            /* Setup the cb and sb patterns. Global route graphs can't depopulate cb and sb
             * since this is a property of a detailed route. */
            seg_details[cur_track].cb = std::make_unique<bool[]>(length);
            seg_details[cur_track].sb = std::make_unique<bool[]>(length+1);
            for (j = 0; j < length; ++j) {
                if (is_global_graph || seg_details[cur_track].longline) {
                    seg_details[cur_track].cb[j] = true;
                } else {
                    /* Use the segment's pattern. */
                    index = j % segment_inf[i].cb.size();
                    seg_details[cur_track].cb[j] = segment_inf[i].cb[index];
                }
            }
            for (j = 0; j < (length + 1); ++j) {
                if (is_global_graph || seg_details[cur_track].longline) {
                    seg_details[cur_track].sb[j] = true;
                } else {
                    /* Use the segment's pattern. */
                    index = j % segment_inf[i].sb.size();
                    seg_details[cur_track].sb[j] = segment_inf[i].sb[index];
                }
            }

            seg_details[cur_track].Rmetal = segment_inf[i].Rmetal;
            seg_details[cur_track].Cmetal = segment_inf[i].Cmetal;
            //seg_details[cur_track].Cmetal_per_m = segment_inf[i].Cmetal_per_m;

            seg_details[cur_track].arch_wire_switch = arch_wire_switch;
            seg_details[cur_track].arch_opin_switch = arch_opin_switch;

            if (BI_DIRECTIONAL == directionality) {
                seg_details[cur_track].direction = BI_DIRECTION;
            } else {
                VTR_ASSERT(UNI_DIRECTIONAL == directionality);
                seg_details[cur_track].direction =
                        (itrack % 2) ? DEC_DIRECTION : INC_DIRECTION;
            }

            seg_details[cur_track].index = i;

            ++cur_track;
        }
    } /* End for each segment type. */

    /* free variables */
    vtr::free(sets_per_seg_type);

    if (num_seg_details) {
        *num_seg_details = cur_track;
    }
    return seg_details;
}

t_chan_details init_chan_details(
        const DeviceGrid& grid,
        const t_chan_width* nodes_per_chan,
        const int num_seg_details,
        const t_seg_details* seg_details,
        const enum e_seg_details_type seg_details_type) {

    VTR_ASSERT(num_seg_details <= nodes_per_chan->get_max());

    t_chan_details chan_details({grid.width(), grid.height(), size_t(num_seg_details)});

    for (size_t x = 0; x < grid.width(); ++x) {
        for (size_t y = 0; y < grid.height(); ++y) {

            t_chan_seg_details* p_seg_details = chan_details[x][y].data();
            for (int i = 0; i < num_seg_details; ++i) {

                p_seg_details[i] = t_chan_seg_details(&seg_details[i]);

                int seg_start = -1;
                int seg_end = -1;

                if (seg_details_type == SEG_DETAILS_X) {
                    seg_start = get_seg_start(p_seg_details, i, y, x);
                    seg_end = get_seg_end(p_seg_details, i, seg_start, y, grid.width() - 2); //-2 for no perim channels
                }
                if (seg_details_type == SEG_DETAILS_Y) {
                    seg_start = get_seg_start(p_seg_details, i, x, y);
                    seg_end = get_seg_end(p_seg_details, i, seg_start, x, grid.height() - 2); //-2 for no perim channels
                }

                p_seg_details[i].set_seg_start(seg_start);
                p_seg_details[i].set_seg_end(seg_end);

                if (seg_details_type == SEG_DETAILS_X) {
                    if (i >= nodes_per_chan->get_x_list_member(y)) {
                        p_seg_details[i].set_length(0);
                    }
                }
                if (seg_details_type == SEG_DETAILS_Y) {
                    if (i >= nodes_per_chan->get_y_list_member(x)) {
                        p_seg_details[i].set_length(0);
                    }
                }
            }
        }
    }
    return chan_details;
}

void adjust_seg_details(
        const int x, const int y,
        const DeviceGrid& grid,
        const t_chan_width* nodes_per_chan,
        t_chan_details& chan_details,
        const enum e_seg_details_type seg_details_type) {

    int seg_index = (seg_details_type == SEG_DETAILS_X ? x : y);

    for (int track = 0; track < nodes_per_chan->get_max(); ++track) {

        int lx = (seg_details_type == SEG_DETAILS_X ? x - 1 : x);
        int ly = (seg_details_type == SEG_DETAILS_X ? y : y - 1);
        if (lx < 0 || ly < 0 || chan_details[lx][ly][track].length() == 0)
            continue;

        while (chan_details[lx][ly][track].seg_end() >= seg_index) {
            chan_details[lx][ly][track].set_seg_end(seg_index - 1);
            lx = (seg_details_type == SEG_DETAILS_X ? lx - 1 : lx);
            ly = (seg_details_type == SEG_DETAILS_X ? ly : ly - 1);
            if (lx < 0 || ly < 0 || chan_details[lx][ly][track].length() == 0)
                break;
        }
    }

    for (int track = 0; track < nodes_per_chan->get_max(); ++track) {

        size_t lx = (seg_details_type == SEG_DETAILS_X ? x + 1 : x);
        size_t ly = (seg_details_type == SEG_DETAILS_X ? y : y + 1);
        if (lx > grid.width() - 2 || ly > grid.height() - 2 || chan_details[lx][ly][track].length() == 0) //-2 for no perim channels
            continue;

        while (chan_details[lx][ly][track].seg_start() <= seg_index) {
            chan_details[lx][ly][track].set_seg_start(seg_index + 1);
            lx = (seg_details_type == SEG_DETAILS_X ? lx + 1 : lx);
            ly = (seg_details_type == SEG_DETAILS_X ? ly : ly + 1);
            if (lx > grid.width() - 2 || ly > grid.height() - 2 || chan_details[lx][ly][track].length() == 0) //-2 for no perim channels
                break;
        }
    }
}


void obstruct_chan_details(
        const DeviceTypes& device_types,
        const DeviceGrid& grid,
        const t_chan_width* nodes_per_chan,
        const bool trim_empty_channels,
        const bool trim_obs_channels,
        t_chan_details& chan_details_x,
        t_chan_details& chan_details_y) {

    /* Iterate grid to find and obstruct based on multi-width/height blocks */
    for (size_t x = 0; x < grid.width() - 1; ++x) {
        for (size_t y = 0; y < grid.height() - 1; ++y) {

            if (!trim_obs_channels)
                continue;

            if (device_types.is_empty_type(grid[x][y].type))
                continue;
            if (grid[x][y].width_offset > 0 || grid[x][y].height_offset > 0)
                continue;
            if (grid[x][y].type->width == 1 && grid[x][y].type->height == 1)
                continue;

            if (grid[x][y].type->height > 1) {
                for (int dx = 0; dx <= grid[x][y].type->width - 1; ++dx) {
                    for (int dy = 0; dy < grid[x][y].type->height - 1; ++dy) {
                        for (int track = 0; track < nodes_per_chan->get_max(); ++track) {
                            chan_details_x[x + dx][y + dy][track].set_length(0);
                        }
                    }
                }
            }
            if (grid[x][y].type->width > 1) {
                for (int dy = 0; dy <= grid[x][y].type->height - 1; ++dy) {
                    for (int dx = 0; dx < grid[x][y].type->width - 1; ++dx) {
                        for (int track = 0; track < nodes_per_chan->get_max(); ++track) {
                            chan_details_y[x + dx][y + dy][track].set_length(0);
                        }
                    }
                }
            }
        }
    }

    /* Iterate grid again to find and obstruct based on neighboring EMPTY and/or IO types */
    for (size_t x = 0; x <= grid.width() - 2; ++x) { //-2 for no perim channels
        for (size_t y = 0; y <= grid.height() - 2; ++y) { //-2 for no perim channels

            if (!trim_empty_channels)
                continue;

            if (device_types.is_io_type(grid[x][y].type)) {
                if ((x == 0) || (y == 0))
                    continue;
            }
            if (device_types.is_empty_type(grid[x][y].type)) {
                if ((x == grid.width() - 2) && device_types.is_io_type(grid[x + 1][y].type)) //-2 for no perim channels
                    continue;
                if ((y == grid.height() - 2) && device_types.is_io_type(grid[x][y + 1].type)) //-2 for no perim channels
                    continue;
            }

            if (device_types.is_io_type(grid[x][y].type) || device_types.is_empty_type(grid[x][y].type)) {
                if (device_types.is_io_type(grid[x][y + 1].type) || device_types.is_empty_type(grid[x][y + 1].type)) {
                    for (int track = 0; track < nodes_per_chan->get_max(); ++track) {
                        chan_details_x[x][y][track].set_length(0);
                    }
                }
            }
            if (device_types.is_io_type(grid[x][y].type) || device_types.is_empty_type(grid[x][y].type)) {
                if (device_types.is_io_type(grid[x + 1][y].type) || device_types.is_empty_type(grid[x + 1][y].type)) {
                    for (int track = 0; track < nodes_per_chan->get_max(); ++track) {
                        chan_details_y[x][y][track].set_length(0);
                    }
                }
            }
        }
    }
}

void adjust_chan_details(
        const DeviceGrid& grid,
        const t_chan_width* nodes_per_chan,
        t_chan_details& chan_details_x,
        t_chan_details& chan_details_y) {

    for (size_t y = 0; y <= grid.height() - 2; ++y) { //-2 for no perim channels
        for (size_t x = 0; x <= grid.width() - 2; ++x) { //-2 for no perim channels

            /* Ignore any non-obstructed channel seg_detail structures */
            if (chan_details_x[x][y][0].length() > 0)
                continue;

            adjust_seg_details(x, y, grid, nodes_per_chan,
                    chan_details_x, SEG_DETAILS_X);
        }
    }

    for (size_t x = 0; x <= grid.width() - 2; ++x) { //-2 for no perim channels
        for (size_t y = 0; y <= grid.height() - 2; ++y) { //-2 for no perim channels

            /* Ignore any non-obstructed channel seg_detail structures */
            if (chan_details_y[x][y][0].length() > 0)
                continue;

            adjust_seg_details(x, y, grid, nodes_per_chan,
                    chan_details_y, SEG_DETAILS_Y);
        }
    }
}


/* Allocates and loads the chan_details data structure, a 2D array of
   seg_details structures. This array is used to handle unique seg_details
   (ie. channel segments) for each horizontal and vertical channel. */

void alloc_and_load_chan_details(
        const DeviceTypes& device_types,
        const DeviceGrid& grid,
        const t_chan_width* nodes_per_chan,
        const bool trim_empty_channels,
        const bool trim_obs_channels,
        const int num_seg_details,
        const t_seg_details* seg_details,
        t_chan_details& chan_details_x,
        t_chan_details& chan_details_y) {

    chan_details_x = init_chan_details(grid, nodes_per_chan,
            num_seg_details, seg_details, SEG_DETAILS_X);
    chan_details_y = init_chan_details(grid, nodes_per_chan,
            num_seg_details, seg_details, SEG_DETAILS_Y);

    /* Obstruct channel segment details based on grid block widths/heights */
    obstruct_chan_details(device_types, grid, nodes_per_chan,
            trim_empty_channels, trim_obs_channels,
            chan_details_x, chan_details_y);

    /* Adjust segment start/end based on obstructed channels, if any */
    adjust_chan_details(grid, nodes_per_chan,
            chan_details_x, chan_details_y);
}

void free_chan_details(
        t_chan_details& chan_details_x,
        t_chan_details& chan_details_y) {

    chan_details_x.clear();
    chan_details_y.clear();
}

