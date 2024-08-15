#include "netlist.h"

/**
 * Regenerate intra-cluster routing in the packer ctx from flat routing results.
 * This function SHOULD be run ONLY when routing is finished!!!
 */
void sync_netlists_to_routing_flat(void);
