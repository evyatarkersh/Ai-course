def h_astar(self, node):
        state = node.state
        grid_size, walls, taps_data, plants_data, robots_data = state
        
        # 1. Calculate Base Action Cost (Total LOADs + Total POURs)
        total_wu_needed = sum(wu for (coord, wu) in plants_data)
        total_wu_carried = sum(load for (id, (r, c, load, capacity)) in robots_data)
        
        # WU Net Needed: The amount of water we still need to load from the taps
        wu_net_needed = max(0, total_wu_needed - total_wu_carried)
        
        # H_Base: Total number of LOADs and POURs required. This is the new admissible base.
        # Total POURs = total_wu_needed
        # Total LOADs = wu_net_needed
        h_base = total_wu_needed + wu_net_needed 
        
        if total_wu_needed == 0:
            return 0
        
        # 2. Calculate Movement Cost Lower Bound (H_Move)
        
        # If there's no need to load more water, movement cost is solely R->P.
        if wu_net_needed == 0:
            # Calculate minimum cost to POUR the carried water. (Min R -> P)
            
            # We only need to consider robots that actually carry water.
            robots_with_water = [(r, c) for (id, (r, c, load, capacity)) in robots_data if load > 0]
            needed_plants = [coord for (coord, wu) in plants_data if wu > 0]

            if not robots_with_water or not needed_plants:
                # If no movement is possible (robots on plants or no plants left), cost is 0
                return h_base

            # Calculate the minimal distance for any robot with water to reach any needed plant.
            min_R_P_dist = min(
                (abs(r_r - p_r) + abs(r_c - p_c))
                for (r_r, r_c) in robots_with_water
                for (p_r, p_c) in needed_plants
            )
            # We assume the robot can deliver ALL the required water in one trip, 
            # so we only add this minimal distance once.
            return h_base + min_R_P_dist


        # 3. If wu_net_needed > 0: We must calculate Min Trips (T->P)
        
        available_taps = [coord for (coord, wu) in taps_data if wu > 0]
        needed_plants = [coord for (coord, wu) in plants_data if wu > 0]
        
        # Fallback if we need water but the tap is empty (unsolvable in this simplified model, but we need admissible lower bound)
        if not available_taps:
            return h_base # We only count the base actions.

        # 3a. Min Movement Cost (R -> T -> P)
        min_trip_cost = utils.infinity
        
        for robot_id, (r, c, load, capacity) in robots_data:
            robot_coord = (r, c)
            
            # Calculate the minimum R -> T -> P path for THIS specific robot
            min_R_T_P_for_this_robot = utils.infinity
            
            for tap_coord in available_taps:
                dist_R_to_T = abs(robot_coord[0] - tap_coord[0]) + abs(robot_coord[1] - tap_coord[1])
                min_dist_T_to_P = min(
                    (abs(tap_coord[0] - plant_coord[0]) + abs(tap_coord[1] - plant_coord[1]))
                    for plant_coord in needed_plants
                )
                path_cost = dist_R_to_T + min_dist_T_to_P
                min_R_T_P_for_this_robot = min(min_R_T_P_for_this_robot, path_cost)
                
            min_trip_cost = min(min_trip_cost, min_R_T_P_for_this_robot)

        # 3b. Min Trips Required (The Admissible Multiplier)
        
        # Find the largest capacity among all robots
        max_capacity = max(capacity for (id, (r, c, load, capacity)) in robots_data)
        
        # The number of WU that MUST be moved (Net Needed) divided by the largest capacity
        min_trips = math.ceil(wu_net_needed / max_capacity)

        # 4. Final Admissible Heuristic: H_Base + H_Move
        # H_Move: Min number of trips multiplied by the min movement cost per trip.
        movement_cost_lower_bound = min_trips * min_trip_cost
        
        return h_base + movement_cost_lower_bound


def h_astar_nextStep(self, node):
        """
    Final Admissible Heuristic for A* (Simplified H_Move).
    Cost = (Min LOAD/POUR actions) + (Min Movement Cost for the Next Necessary Step).
    """
        state = node.state
        grid_size, walls, taps_data, plants_data, robots_data = state
        
        # --- 1. Base Action Cost (The Admissible Core) ---
        total_wu_needed = sum(wu for (coord, wu) in plants_data)
        total_wu_carried = sum(load for (id, (r, c, load, capacity)) in robots_data)
        wu_net_needed = max(0, total_wu_needed - total_wu_carried)
        h_base = total_wu_needed + wu_net_needed  # This part is Admissible!
        
        if total_wu_needed == 0:
            return 0
        
        # --- 2. H_Move: Min Cost to Achieve the Next Goal (The safest lower bound) ---
        
        needed_plants = [coord for (coord, wu) in plants_data if wu > 0]
        
        # Find the target locations needed for the next step (Taps or Plants)
        potential_targets = []

        # If water is still needed (wu_net_needed > 0), the next goal MUST involve a Tap.
        if wu_net_needed > 0:
            available_taps = [coord for (coord, wu) in taps_data if wu > 0]
            potential_targets.extend(available_taps)
        
        # If there are plants that need water, the next goal MUST involve a Plant.
        if needed_plants:
            potential_targets.extend(needed_plants)

        # Calculate Min Movement Cost
        min_dist_to_target = utils.infinity

        for robot_id, (r, c, load, capacity) in robots_data:
            robot_coord = (r, c)
            
            # Determine the target set for this specific robot:
            current_targets = []

            # A. If robot has load, goal is to POUR (Plant)
            if load > 0:
                current_targets.extend(needed_plants)
                
            # B. If robot has no load (or needs more), and we need more water overall, goal is to LOAD (Tap)
            if (load == 0 and wu_net_needed > 0) or (load < capacity and wu_net_needed > 0):
                current_targets.extend(available_taps)

            # Calculate minimum distance from the robot to any of its potential targets
            for target_coord in set(current_targets): # Use set to avoid duplicates
                dist = abs(robot_coord[0] - target_coord[0]) + abs(robot_coord[1] - target_coord[1])
                min_dist_to_target = min(min_dist_to_target, dist)

        # If no movement is required (min_dist_to_target is 0, e.g., robot on target), we add 0.
        # We add this minimal distance only once, as a lower bound on the cost of the *first* needed move.
        
        if min_dist_to_target == utils.infinity:
            # This shouldn't happen if needed_plants > 0, but as a safe guard:
            return h_base 

        return h_base + min_dist_to_target