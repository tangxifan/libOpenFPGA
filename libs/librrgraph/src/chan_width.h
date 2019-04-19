/* IMPORTANT:
 * The following preprocessing flags are added to 
 * avoid compilation error when this headers are included in more than 1 times 
 */
#ifndef CHAN_WIDTH_H
#define CHAN_WIDTH_H

#include "device_grid.h"

/* Class Channel width */
class t_chan_width {
  /* Initialization functions, frequently used by users  */
  public:
    void init(const DeviceGrid& grid, int cfactor, t_chan_width_dist chan_width_dist);
  /* Basic data read/write function */
  public:
    /* Read functions */
    int get_max() const { return max_; }
    int get_x_max() const { return x_max_; } 
    int get_x_min() const { return x_min_; } 
    int get_y_max() const { return y_max_; } 
    int get_y_min() const { return y_min_; } 
    std::vector<int> get_x_list() const { return x_list_;}
    std::vector<int> get_y_list() const { return y_list_;}
    int get_x_list_member(int index) const { return x_list_[index];}
    int get_y_list_member(int index) const { return y_list_[index];}
    /* Write functions */
    void set_max(int max_i) { max_ = max_i; }
    void set_x_max(int x_max_i)  { x_max_ = x_max_i; } 
    void set_x_min(int x_min_i)  { x_min_ = x_min_i; } 
    void set_y_max(int y_max_i)  { y_max_ = y_max_i; } 
    void set_y_min(int y_min_i)  { y_min_ = y_min_i; } 
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

