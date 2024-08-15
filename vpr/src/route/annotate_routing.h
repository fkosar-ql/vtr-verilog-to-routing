#ifndef ANNOTATE_ROUTING_H
#define ANNOTATE_ROUTING_H

/********************************************************************
 * Include header files that are required by function declaration
 *******************************************************************/
#include "vpr_context.h"

/********************************************************************
 * Function declaration
 *******************************************************************/

vtr::vector<RRNodeId, ClusterNetId> annotate_rr_node_nets(const ClusteringContext& cluster_ctx,
                                                         const DeviceContext& device_ctx,
                                                         const bool& verbose);

#endif
