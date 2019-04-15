/* IMPORTANT:
 * The following preprocessing flags are added to 
 * avoid compilation error when this headers are included in more than 1 times 
 */
#ifndef CHAN_WIDTH_H
#define CHAN_WIDTH_H

#include "context.h"
#include "device_grid.h"

/* Class Channel width */
class t_chan_width : public Context {
  /* Initialization functions, frequently used by users  */
  public:
    void init(const DeviceGrid& grid, int cfactor, t_chan_width_dist chan_width_dist) {
  /* Basic data read/write function */
  public:
    /* Read functions */
    int get_max() const { return max_; }
    int get_x_max() const { return x_max_; } 
    int get_x_min() const { return x_min_; } 
    int get_y_max() const { return y_max_; } 
    int get_y_min() const { return y_min_; } 
    /* Write functions */
    int set_max(int max) { max_ = max; }
    int set_x_max(int x_max)  { x_max_ = x_max; } 
    int set_x_min(int x_min)  { x_min_ = x_min; } 
    int set_y_max(int y_max)  { y_max_ = y_max; } 
    int set_y_min(int y_min)  { y_min_ = y_min; } 
  /* All members are private, not accessible directly from outside */
  private:
	int max_ = 0;
	int x_max_ = 0;
	int y_max_ = 0;
	int x_min_ = 0;
	int y_min_ = 0;
    std::vector<int> x_list_;
	std::vector<int> y_list_;
};

#endif

