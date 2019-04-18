/* IMPORTANT:
 * The following preprocessing flags are added to 
 * avoid compilation error when this headers are included in more than 1 times 
 */
#ifndef RR_INDEXED_DATA_H
#define RR_INDEXED_DATA_H


/* Data that is pointed to by the .cost_index member of t_rr_node.  It's     *
 * purpose is to store the base_cost so that it can be quickly changed       *
 * and to store fields that have only a few different values (like           *
 * seg_index) or whose values should be an average over all rr_nodes of a    *
 * certain type (like T_linear etc., which are used to predict remaining     *
 * delay in the timing_driven router).                                       *
 *                                                                           *
 * base_cost:  The basic cost of using an rr_node.                           *
 * ortho_cost_index:  The index of the type of rr_node that generally        *
 *                    connects to this type of rr_node, but runs in the      *
 *                    orthogonal direction (e.g. vertical if the direction   *
 *                    of this member is horizontal).                         *
 * seg_index:  Index into segment_inf of this segment type if this type of   *
 *             rr_node is an CHANX or CHANY; OPEN (-1) otherwise.            *
 * inv_length:  1/length of this type of segment.                            *
 * T_linear:  Delay through N segments of this type is N * T_linear + N^2 *  *
 *            T_quadratic.  For buffered segments all delay is T_linear.     *
 * T_quadratic:  Dominant delay for unbuffered segments, 0 for buffered      *
 *               segments.                                                   *
 * C_load:  Load capacitance seen by the driver for each segment added to    *
 *          the chain driven by the driver.  0 for buffered segments.        */
class t_rr_indexed_data : public Context{
  /* Methods to create/free/access/modify each member */
  public:
    /* Constructors */
    t_rr_indexed_data(0., 0., -1, -1, 0., 0., 0., 0.);
    void init(float, float, int, int, float, float, float);

    /* Basic data read/write function */ 
    float base_cost() const { return base_cost_; }
    float mutable_base_cost() { return base_cost_; }    
    float get_base_cost() { return base_cost_; }    
    void  set_base_cost(float tmp) { base_cost_ = tmp; return; }

    float saved_base_cost() const { return saved_base_cost_; }
    float mutable_saved_base_cost() { return saved_base_cost_; }    
    float get_saved_base_cost() { return saved_base_cost_; }    
    void  set_saved_base_cost(float tmp) { saved_base_cost_ = tmp; return; }

    int  ortho_cost_index() const { return ortho_base_index_; }
    int  mutable_ortho_cost_index() { return ortho_cost_index_; }    
    int  get_ortho_cost_index() { return ortho_cost_index_; }    
    void set_ortho_cost_index(int tmp) { ortho_cost_index_ = tmp; return; }

    int  seg_index() const { return seg_index_; }
    int  mutable_seg_index() { return seg_index_; }    
    int  get_seg_index() { return seg_index_; }    
    void set_seg_index(int tmp) { seg_index_ = tmp; return; }

    float inv_length() const { return inv_length_; }
    float mutable_inv_length() { return inv_length_; }    
    float get_inv_length() { return inv_length_; }    
    void  set_inv_length(float tmp) { inv_length_ = tmp; return; }

    float T_linear() const { return T_linear_; }
    float mutable_T_linear() { return T_linear_; }    
    float get_T_linear() { return T_linear_; }    
    void  set_T_linear(float tmp) { T_linear_ = tmp; return; }

    float T_quadratic() const { return T_quadratic_; }
    float mutable_T_quadratic() { return T_quadratic_; }    
    float get_T_quadratic() { return T_quadratic_; }
    void  set_T_quadratic(float tmp) { T_quadratic_ = tmp; return; }

    float C_load() const { return C_load_; }
    float mutable_C_load() { return C_load_; }    
    float get_C_load() { return C_load_; }
    void  set_C_load(float tmp) { C_load_ = tmp; return; }

  /* Private data only accessible by the class functions */ 
  private:
	float base_cost_;
	float saved_base_cost_;
	int ortho_cost_index_;
	int seg_index_;
	float inv_length_;
	float T_linear_;
	float T_quadratic_;
	float C_load_;
};

#endif
