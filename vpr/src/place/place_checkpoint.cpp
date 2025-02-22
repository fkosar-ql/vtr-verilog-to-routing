#include "place_checkpoint.h"
#include "noc_place_utils.h"

float t_placement_checkpoint::get_cp_cpd() { return cpd; }
double t_placement_checkpoint::get_cp_bb_cost() { return costs.bb_cost; }
bool t_placement_checkpoint::cp_is_valid() { return valid; }

void t_placement_checkpoint::save_placement(const t_placer_costs& placement_costs, const float& critical_path_delay) {
    auto& place_ctx = g_vpr_ctx.placement();
    block_locs = place_ctx.block_locs;
    valid = true;
    cpd = critical_path_delay;
    costs = placement_costs;
}

t_placer_costs t_placement_checkpoint::restore_placement() {
    auto& mutable_place_ctx = g_vpr_ctx.mutable_placement();
    mutable_place_ctx.block_locs = block_locs;
    load_grid_blocks_from_block_locs();
    return costs;
}

void save_placement_checkpoint_if_needed(t_placement_checkpoint& placement_checkpoint, std::shared_ptr<SetupTimingInfo> timing_info, t_placer_costs& costs, float cpd) {
    if (placement_checkpoint.cp_is_valid() == false || (timing_info->least_slack_critical_path().delay() < placement_checkpoint.get_cp_cpd() && costs.bb_cost <= placement_checkpoint.get_cp_bb_cost())) {
        placement_checkpoint.save_placement(costs, cpd);
        VTR_LOG("Checkpoint saved: bb_costs=%g, TD costs=%g, CPD=%7.3f (ns) \n", costs.bb_cost, costs.timing_cost, 1e9 * cpd);
    }
}

void restore_best_placement(t_placement_checkpoint& placement_checkpoint,
                            std::shared_ptr<SetupTimingInfo>& timing_info,
                            t_placer_costs& costs,
                            std::unique_ptr<PlacerCriticalities>& placer_criticalities,
                            std::unique_ptr<PlacerSetupSlacks>& placer_setup_slacks,
                            std::unique_ptr<PlaceDelayModel>& place_delay_model,
                            std::unique_ptr<NetPinTimingInvalidator>& pin_timing_invalidator,
                            PlaceCritParams crit_params,
                            const t_noc_opts& noc_opts) {
    /* The (valid) checkpoint is restored if the following conditions are met:
     * 1) The checkpoint has a lower critical path delay.
     * 2) The checkpoint's wire-length cost is either better than the current solution,
     * or at least is not more than 5% worse than the current solution.
     */
    if (placement_checkpoint.cp_is_valid() && timing_info->least_slack_critical_path().delay() > placement_checkpoint.get_cp_cpd() && costs.bb_cost * 1.05 > placement_checkpoint.get_cp_bb_cost()) {
        //restore the latest placement checkpoint
        costs = placement_checkpoint.restore_placement();

        //recompute timing from scratch
        placer_criticalities.get()->set_recompute_required();
        placer_setup_slacks.get()->set_recompute_required();
        comp_td_connection_delays(place_delay_model.get());
        perform_full_timing_update(crit_params,
                                   place_delay_model.get(),
                                   placer_criticalities.get(),
                                   placer_setup_slacks.get(),
                                   pin_timing_invalidator.get(),
                                   timing_info.get(),
                                   &costs);

        /* If NoC is enabled, re-compute NoC costs and re-initialize NoC internal data structures.
         * If some routers have different locations than the last placement, NoC-related costs and
         * internal data structures that are used to keep track of each flow's cost are no longer valid,
         * and need to be re-computed from scratch.
         */
        if (noc_opts.noc) {
            reinitialize_noc_routing(costs, {});
        }

        VTR_LOG("\nCheckpoint restored\n");
    }
}
