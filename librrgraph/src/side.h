#ifndef SIDE_H
#define SIDE_H 
/* Generic Orientations of a Routing Resouce Node (rr_node) 
 * Only applicable to rr_node whose type is 
 * IPIN or OPIN in e_rr_type
 * i.e., input/output of logic blocks
 * 1. TOP/RIGHT/BOTTOM/RIGHT: 
 *    location of a IPIN/OPIN on the border of a logic block (CLB, etc.)
 * 2. NUM_SIDES:
 *    A quick counter for number of sides in the enumeration
 */
enum e_side : unsigned char {
	TOP = 0,
    RIGHT = 1,
    BOTTOM = 2,
    LEFT = 3,
    NUM_SIDES
};

/* Constant expression to save memory */
constexpr std::array<e_side, NUM_SIDES> SIDES = { {TOP, RIGHT, BOTTOM, LEFT} }; //Set of all side orientations
/* Constant expression to ease print log files */
constexpr std::array<const char*, NUM_SIDES> SIDE_STRING = { {"TOP", "RIGHT", "BOTTOM", "LEFT"} }; //String versions of side orientations

#endif
