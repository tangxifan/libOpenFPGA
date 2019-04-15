#include "chan_width.h"
#include "device_grid.h"
#include "physical_types.h"

/* Embodiment Functions for class t_chan_width */
/* Constructor for routing channels */
void t_chan_width::init(const DeviceGrid& grid, 
                        int cfactor, 
                        t_chan_width_dist chan_width_dist) {

    /* Assigns widths to channels (in tracks).  Minimum one track          *
     * per channel. The channel distributions read from the architecture  *
     * file are scaled by cfactor.                                         */
    t_chan chan_x_dist = chan_width_dist.chan_x_dist;
    t_chan chan_y_dist = chan_width_dist.chan_y_dist;

    x_list_.resize(grid.height());
    y_list_.resize(grid.width());

    if (grid.height() > 1) {
        int num_channels = grid.height() - 1;
        VTR_ASSERT(num_channels > 0);
        float separation = 1.0 / num_channels; /* Norm. distance between two channels. */

        for (size_t i = 0; i < grid.height(); ++i) {
            float y = float(i) / num_channels;
            x_list_[i] = (int) floor(cfactor * comp_width(&chan_x_dist, y, separation) + 0.5);
            x_list_[i] = max(x_list_[i], 1); //Minimum channel width 1
        }
    }

    if (grid.width() > 1) {
        int num_channels = grid.width() - 1;
        VTR_ASSERT(num_channels > 0);
        float separation = 1.0 / num_channels; /* Norm. distance between two channels. */

        for (size_t i = 0; i < grid.width(); ++i) { //-2 for no perim channels
            float x = float(i) / num_channels;

            y_list_[i] = (int) floor(cfactor * comp_width(&chan_y_dist, x, separation) + 0.5);
            y_list_[i] = max(y_list_[i], 1); //Minimum channel width 1
        }
    }

    max_ = 0;
    x_max_ = y_max_ = INT_MIN;
    x_min_ = y_min_ = INT_MAX;
    for (size_t i = 0; i < grid.height(); ++i) {
        max_ = max(max_, x_list_[i]);
        x_max_ = max(x_max_, x_list_[i]);
        x_min_ = min(x_min_, x_list_[i]);
    }
    for (size_t i = 0; i < grid.width(); ++i) {
        max_ = max(max_, y_list_[i]);
        y_max_ = max(y_max_, y_list_[i]);
        y_min_ = min(y_min_, y_list_[i]);
    }

#ifdef VERBOSE
  VTR_LOG("\n");
  VTR_LOG("chan_width.x_list:\n");
  for (size_t i = 0; i < grid.height(); ++i) {
      VTR_LOG("%d  ", x_list_[i]);
  }
  VTR_LOG("\n");
  VTR_LOG("chan_width.y_list:\n");
  for (size_t i = 0; i < grid.width(); ++i) {
      VTR_LOG("%d  ", y_list_[i]);
  }
  VTR_LOG("\n");
#endif

  return;
}


