/********************************************************************
 * This file includes functions to fix up the pb pin mapping results 
 * after routing optimization
 *******************************************************************/
/* Headers from vtrutil library */
#include "clustered_netlist_utils.h"
#include "logic_types.h"
#include "netlist_fwd.h"
#include "physical_types.h"
#include "vtr_time.h"
#include "vtr_assert.h"
#include "vtr_log.h"

#include "vpr_error.h"
#include "vpr_utils.h"
#include "rr_graph2.h"

#include "annotate_routing.h"

#include "sync_netlists_to_routing_flat.h"
#include <algorithm>
#include <unordered_map>

#include "describe_rr_node.h"

/* Include global variables of VPR */
#include "globals.h"

/* Output all intra-cluster connections for a RouteTreeNode */
void get_intra_cluster_connections(const RouteTree& tree, std::vector<std::pair<RRNodeId, RRNodeId>>& out_connections){
    auto& rr_graph = g_vpr_ctx.device().rr_graph;

    for(auto& node: tree.all_nodes()){
        const auto& parent = node.parent();
        if(!parent) /* Root */
            continue;

        auto type = rr_graph.node_type(node.inode);
        auto parent_type = rr_graph.node_type(parent->inode);

        /* Both nodes are IPIN/OPIN: this has to be an intrablock connection */
        if((type == OPIN || type == IPIN) && (parent_type == OPIN || parent_type == IPIN)){
            out_connections.push_back({parent->inode, node.inode});
        }
    }
}

/** Rudimentary intra-cluster router between two pb_graph pins. 
 * Easier to use than the packer's router, but it assumes that there is only one path between the provided pins.
 * Expect this to fail/produce invalid results if that's not the case with your architecture.
 * Outputs the path to the given pb. */
static void route_intra_cluster_conn(const t_pb_graph_pin* source_pin, const t_pb_graph_pin* sink_pin, AtomNetId net_id, t_pb* out_pb){

    std::unordered_set<const t_pb_graph_pin*> visited;
    std::deque<const t_pb_graph_pin*> queue;
    std::unordered_map<const t_pb_graph_pin*, const t_pb_graph_pin*> prev;

    auto& out_pb_routes = out_pb->pb_route;

    queue.push_back(source_pin);
    prev[source_pin] = NULL;

    while(!queue.empty()){
        const t_pb_graph_pin* cur_pin = queue.front();
        //std::cout << "expanding: " << cur_pin->to_string() << "\n";
        queue.pop_front();
        if(visited.count(cur_pin))
            continue;
        visited.insert(cur_pin);

        /* Backtrack and return */
        if(cur_pin == sink_pin){
            break;
        }

        for(auto& edge: cur_pin->output_edges){
            VTR_ASSERT(edge->num_output_pins == 1);
            queue.push_back(edge->output_pins[0]);
            //std::cout << "pushing back " << edge->output_pins[0]->to_string() << "\n";
            prev[edge->output_pins[0]] = cur_pin;
        }
    }

    VTR_ASSERT_MSG(visited.count(sink_pin), "Couldn't find sink pin");

    /* Collect path: we need to build pb_routes from source to sink */
    std::vector<const t_pb_graph_pin*> path;
    const t_pb_graph_pin* cur_pin = sink_pin;
    while(cur_pin){
        path.push_back(cur_pin);
        cur_pin = prev[cur_pin];
    }

    /* Output the path into out_pb_routes (start from source) */
    int prev_pin_id = -1;
    for(auto it = path.rbegin(); it != path.rend(); ++it){
        cur_pin = *it;
        int cur_pin_id = cur_pin->pin_count_in_cluster;
        t_pb_route* cur_pb_route;

        if(out_pb_routes.count(cur_pin_id))
            cur_pb_route = &out_pb_routes[cur_pin_id];
        else {
            t_pb_route pb_route = {
                net_id,
                -1,
                {},
                cur_pin
            };
            out_pb_routes.insert(std::make_pair<>(cur_pin_id, pb_route));
            cur_pb_route = &out_pb_routes[cur_pin_id];
        }

        if(prev_pin_id != -1){
            t_pb_route& prev_pb_route = out_pb_routes[prev_pin_id];
            prev_pb_route.sink_pb_pin_ids.push_back(cur_pin_id);
            cur_pb_route->driver_pb_pin_id = prev_pb_route.pb_graph_pin->pin_count_in_cluster;
        }

        prev_pin_id = cur_pin_id;
    }
}

void sync_pb_routes_to_routing(){
    auto& device_ctx = g_vpr_ctx.device();
    auto& atom_ctx = g_vpr_ctx.atom();
    auto& cluster_ctx = g_vpr_ctx.mutable_clustering();
    auto& placement_ctx = g_vpr_ctx.placement();
    auto& route_ctx = g_vpr_ctx.routing();
    auto& rr_graph = device_ctx.rr_graph;

    /* Clear out existing pb_routes: they were made by the intra cluster router and are invalid now */
    for (ClusterBlockId clb_blk_id : cluster_ctx.clb_nlist.blocks()) {
        /* Only erase entries which are not associated with a clock net: the router doesn't touch the clock nets
         * XXX: Assumes --clock_modeling ideal */
        std::vector<int> pins_to_erase;
        auto& pb_routes = cluster_ctx.clb_nlist.block_pb(clb_blk_id)->pb_route;
        for(auto& [pin, pb_route]: pb_routes){
            if(!route_ctx.is_clock_net[pb_route.atom_net_id])
                pins_to_erase.push_back(pin);
        }

        for(int pin: pins_to_erase){
            pb_routes.erase(pin);
        }
    }

    /* Go through each route tree and rebuild the pb_routes */
    for(ParentNetId net_id: atom_ctx.nlist.nets()){
        auto& tree = route_ctx.route_trees[net_id];
        if(!tree)
            continue; /* No routing at this ParentNetId */

        /* Get all intrablock connections */
        std::vector<std::pair<RRNodeId, RRNodeId>> conns_to_restore; /* (source, sink) */
        get_intra_cluster_connections(tree.value(), conns_to_restore);

        /* Restore the connections */
        for(auto [source_inode, sink_inode]: conns_to_restore){
            auto physical_tile = device_ctx.grid.get_physical_type({
                rr_graph.node_xlow(source_inode),
                rr_graph.node_ylow(source_inode),
                rr_graph.node_layer(source_inode)
            });
            int source_pin = rr_graph.node_pin_num(source_inode);
            int sink_pin = rr_graph.node_pin_num(sink_inode);

            auto [_, subtile] = get_sub_tile_from_pin_physical_num(physical_tile, source_pin);

            ClusterBlockId clb = placement_ctx.grid_blocks.block_at_location({
                rr_graph.node_xlow(source_inode),
                rr_graph.node_ylow(source_inode),
                subtile,
                rr_graph.node_layer(source_inode)
            });

            /* Look up pb graph pins from pb type if pin is not on tile, look up from block otherwise */
            const t_pb_graph_pin* source_pb_graph_pin, *sink_pb_graph_pin;
            if(is_pin_on_tile(physical_tile, sink_pin)){
                sink_pb_graph_pin = get_pb_graph_node_pin_from_block_pin(clb, sink_pin);
            }else{
                sink_pb_graph_pin = get_pb_pin_from_pin_physical_num(physical_tile, sink_pin);
            }
            if(is_pin_on_tile(physical_tile, source_pin)){
                source_pb_graph_pin = get_pb_graph_node_pin_from_block_pin(clb, source_pin);
            }else{
                source_pb_graph_pin = get_pb_pin_from_pin_physical_num(physical_tile, source_pin);
            }

            t_pb* pb = cluster_ctx.clb_nlist.block_pb(clb);

            /* Route between the pins */
            route_intra_cluster_conn(source_pb_graph_pin, sink_pb_graph_pin, convert_to_atom_net_id(net_id), pb);
        }
    }
}

/** Rebuild the ClusterNetId <-> AtomNetId lookup after compressing the ClusterNetlist
 * Needs the "most recent" ClusterNetIds in atom_ctx.lookup: won't work after invalidating the ClusterNetIds twice */
inline void rebuild_atom_nets_lookup(ClusteredNetlist::IdRemapper& remapped){
    auto& atom_ctx = g_vpr_ctx.mutable_atom();
    auto& atom_lookup = atom_ctx.lookup;

    for(auto parent_net_id: atom_ctx.nlist.nets()){
        auto atom_net_id = convert_to_atom_net_id(parent_net_id);
        ClusterNetId old_clb_net = atom_lookup.clb_net(atom_net_id);
        if(!old_clb_net)
            continue;
        ClusterNetId new_clb_net = remapped.new_net_id(old_clb_net);
        atom_lookup.set_atom_clb_net(atom_net_id, new_clb_net);
    }
}

/** Regenerate clustered netlist nets from routing results */
void sync_clustered_netlist_to_routing(void){
    auto& cluster_ctx = g_vpr_ctx.mutable_clustering();
    auto& place_ctx = g_vpr_ctx.mutable_placement();
    auto& route_ctx = g_vpr_ctx.routing();
    auto& clb_netlist = cluster_ctx.clb_nlist;
    auto& device_ctx = g_vpr_ctx.device();
    auto& rr_graph = device_ctx.rr_graph;
    auto& atom_ctx = g_vpr_ctx.mutable_atom();
    auto& atom_lookup = atom_ctx.lookup;

    /* 1. Remove all nets, pins and ports from the clustered netlist (except clocks) */
    for(auto net_id: clb_netlist.nets()){
        auto atom_net_id = atom_lookup.atom_net(net_id);
        if(route_ctx.is_clock_net[atom_net_id])
            continue;

        clb_netlist.remove_net(net_id);
        atom_lookup.set_atom_clb_net(AtomNetId::INVALID(), net_id);
    }
    for(auto pin_id: clb_netlist.pins()){
        ClusterNetId clb_net_id = clb_netlist.pin_net(pin_id);
        auto atom_net_id = atom_lookup.atom_net(clb_net_id);
        if(atom_net_id && route_ctx.is_clock_net[atom_net_id])
            continue;
        clb_netlist.remove_pin(pin_id);
    }
    for(auto port_id: clb_netlist.ports()){
        ClusterNetId clb_net_id = clb_netlist.port_net(port_id, 0);
        auto atom_net_id = atom_lookup.atom_net(clb_net_id);
        if(atom_net_id && route_ctx.is_clock_net[atom_net_id])
            continue;
        clb_netlist.remove_port(port_id);
    }
    /* 2. Reset all internal lookups for netlist */
    auto remapped = clb_netlist.compress();
    rebuild_atom_nets_lookup(remapped);

    /* 3. Walk each routing in the atom netlist. If a node is on the tile, add a ClusterPinId for it.
     * Add the associated net and port too if they don't exist */
    for(auto parent_net_id: atom_ctx.nlist.nets()){
        auto& tree = route_ctx.route_trees[parent_net_id];
        AtomNetId atom_net_id = convert_to_atom_net_id(parent_net_id);

        ClusterNetId clb_net_id;
        for(auto& rt_node: tree->all_nodes()){
            auto node_type = rr_graph.node_type(rt_node.inode);
            if(node_type != IPIN && node_type != OPIN)
                continue;

            auto physical_tile = device_ctx.grid.get_physical_type({
                rr_graph.node_xlow(rt_node.inode),
                rr_graph.node_ylow(rt_node.inode),
                rr_graph.node_layer(rt_node.inode)
            });

            int pin_index = rr_graph.node_pin_num(rt_node.inode);

            auto [_, subtile] = get_sub_tile_from_pin_physical_num(physical_tile, pin_index);

            ClusterBlockId clb = place_ctx.grid_blocks.block_at_location({
                rr_graph.node_xlow(rt_node.inode),
                rr_graph.node_ylow(rt_node.inode),
                subtile,
                rr_graph.node_layer(rt_node.inode)
            });

            if(!is_pin_on_tile(physical_tile, pin_index))
                continue;

            if(!clb_net_id){
                clb_net_id = clb_netlist.create_net(atom_ctx.nlist.net_name(parent_net_id));
                atom_lookup.set_atom_clb_net(atom_net_id, clb_net_id);
            }

            t_pb_graph_pin* pb_graph_pin = get_pb_graph_node_pin_from_block_pin(clb, pin_index);

            ClusterPortId port_id = clb_netlist.find_port(clb, pb_graph_pin->port->name);
            if(!port_id){
                PortType port_type;
                if(pb_graph_pin->port->is_clock)
                    port_type = PortType::CLOCK;
                else if(pb_graph_pin->port->type == IN_PORT)
                    port_type = PortType::INPUT;
                else if(pb_graph_pin->port->type == OUT_PORT)
                    port_type = PortType::OUTPUT;
                else
                    VTR_ASSERT_MSG(false, "Unsupported port type");
                port_id = clb_netlist.create_port(clb, pb_graph_pin->port->name, pb_graph_pin->port->num_pins, port_type);
            }
            PinType pin_type = node_type == OPIN ? PinType::DRIVER : PinType::SINK;

            ClusterPinId new_pin = clb_netlist.create_pin(port_id, pb_graph_pin->pin_number, clb_net_id, pin_type, pb_graph_pin->pin_count_in_cluster);
            clb_netlist.set_pin_net(new_pin, pin_type, clb_net_id);
        }
    }
    /* 4. Rebuild internal cluster netlist lookups */
    remapped = clb_netlist.compress();
    rebuild_atom_nets_lookup(remapped);
    /* 5. Rebuild place_ctx.physical_pins lookup
     * TODO: maybe we don't need this fn and pin_index is enough? */
    place_ctx.physical_pins.clear();
    for(auto clb: clb_netlist.blocks()){
        place_sync_external_block_connections(clb);
    }
    /* TODO: Remove rr_nodes added by the flat router */
}

/** Fix up pin rotation maps and the atom pin -> pb graph pin lookup for every block */
void fixup_atom_pb_graph_pin_mapping(void){
    auto& cluster_ctx = g_vpr_ctx.clustering();
    auto& atom_ctx = g_vpr_ctx.mutable_atom();

    for(ClusterBlockId clb: cluster_ctx.clb_nlist.blocks()){    
        /* Collect all innermost pb routes */
        std::vector<int> sink_pb_route_ids;
        t_pb* clb_pb = cluster_ctx.clb_nlist.block_pb(clb);
        for(auto [pb_route_id, pb_route]: clb_pb->pb_route){
            if(pb_route.sink_pb_pin_ids.empty())
                sink_pb_route_ids.push_back(pb_route_id);
        }

        for(int sink_pb_route_id: sink_pb_route_ids){
            t_pb_route& pb_route = clb_pb->pb_route.at(sink_pb_route_id);

            const t_pb_graph_pin* atom_pbg_pin = pb_route.pb_graph_pin;
            t_pb* atom_pb = clb_pb->find_mutable_pb(atom_pbg_pin->parent_node);
            AtomBlockId atb = atom_ctx.lookup.pb_atom(atom_pb);
            if(!atb)
                continue;

            /* Find atom port from pbg pin's model port */
            AtomPortId atom_port = atom_ctx.nlist.find_atom_port(atb, atom_pbg_pin->port->model_port);
            for(AtomPinId atom_pin: atom_ctx.nlist.port_pins(atom_port)){
                /* Match net IDs from pb_route and atom netlist and connect in lookup */
                if(pb_route.atom_net_id == atom_ctx.nlist.pin_net(atom_pin)){
                    atom_ctx.lookup.set_atom_pin_pb_graph_pin(atom_pin, atom_pbg_pin);
                    atom_pb->set_atom_pin_bit_index(atom_pbg_pin, atom_ctx.nlist.pin_port_bit(atom_pin));
                }
            }
        }
    }
}

/**
 * Regenerate intra-cluster routing in the packer ctx from flat routing results.
 * This function SHOULD be run ONLY when routing is finished!!!
 */
void sync_netlists_to_routing_flat(void) {
    vtr::ScopedStartFinishTimer timer("Synchronize the packed netlist to routing optimization");

    sync_clustered_netlist_to_routing();
    sync_pb_routes_to_routing();
    fixup_atom_pb_graph_pin_mapping();
}
