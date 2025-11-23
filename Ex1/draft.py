def h_astar_refined(self, node):
    """
    This is an admissible heuristic for A* that incorporates movement cost 
    by calculating the minimum number of necessary trips (LOAD/POUR cycles) 
    and multiplying that by the minimum R->T->P path cost.
    
    This is MORE informative than WU * 2, while remaining admissible.
    """
    state = node.state
    grid_size, walls, taps_data, plants_data, robots_data = state
    robots_map = dict(robots_data) 

    # 1. Base Cost & Total WU Needed
    total_wu_needed = sum(wu for (coord, wu) in plants_data)
    if total_wu_needed == 0:
        return 0
    
    available_taps = [coord for (coord, wu) in taps_data if wu > 0]
    needed_plants = [coord for (coord, wu) in plants_data if wu > 0]

    # Fallback to WU * 2 if movement cost cannot be calculated
    if not available_taps or not needed_plants:
        return total_wu_needed * 2 

    # 2. Find Min Trip Cost (Min R -> T -> P)
    # This part is identical to the GBFS calculation, finding the cheapest cost 
    # for ANY robot to complete ONE full cycle.
    min_total_path_cost = utils.infinity
        
    # Iterate over all robots
    for robot_id, (r, c, load, capacity) in robots_map.items():
        robot_coord = (r, c)
        
        # Calculate the minimum R -> T -> P path for THIS specific robot
        min_R_T_P_for_this_robot = utils.infinity
        
        # Iterate over all available Taps
        for tap_coord in available_taps:
            
            # a. Distance R to this T (LOAD cost)
            # Manhattan Distance calculation
            dist_R_to_T = abs(robot_coord[0] - tap_coord[0]) + abs(robot_coord[1] - tap_coord[1])
            
            # b. Min distance from this specific T to ANY plant P (POUR cost)
            # Find the best plant target for THIS tap
            min_dist_T_to_P = min(
                (abs(tap_coord[0] - plant_coord[0]) + abs(tap_coord[1] - plant_coord[1]))
                for plant_coord in needed_plants
            )
            
            # c. Total path cost through this T (for THIS robot)
            path_cost = dist_R_to_T + min_dist_T_to_P
            
            # Update the minimum path cost found for the CURRENT robot
            min_R_T_P_for_this_robot = min(min_R_T_P_for_this_robot, path_cost)
            
        # Update the overall minimum trip cost across ALL robots
        min_total_path_cost = min(min_total_path_cost, min_R_T_P_for_this_robot)


    # 3. Min Trips Required (The Admissible Multiplier)
    # The minimum number of trips needed is the total water divided by the 
    # maximum capacity of ANY robot (optimistic/lower bound).
    max_capacity = max(capacity for (id, (r, c, load, capacity)) in robots_data)
    
    # Calculate the MINIMUM number of trips required (ceiling function)
    min_trips = math.ceil(total_wu_needed / max_capacity)

    # 4. Final Admissible Heuristic:
    # Cost = (Base LOAD/POUR actions) + (Min Trips * Min Movement Cost per trip)
    # Since total_wu_needed is the number of total POURs/LOADs, 
    # we only add the movement cost.
    
    movement_cost_lower_bound = min_trips * min_trip_cost
    
    return total_wu_needed * 2 + movement_cost_lower_bound



def h_astarPreviousLast(self, node):
        print("arrived to h_astar in ex1.py!!!!!!!!!!!!!! ")
        """ This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""
        """
        This is the admissible heuristic for A*. It lower-bounds the remaining cost (h(s)).
        
        Heuristic: Total Water Needed * 2. 
        (Relaxation: Ignores all movement cost, wall/robot conflicts, and capacity limits).
        """
    
        # Extract the state tuple from the node
        state = node.state
        
        # Extract Plants data (index 3): frozenset of ((r, c), wu_needed)
        plants_data = state[3]
        
        total_wu_needed = 0
        
        # Calculate the total amount of water still required by all plants
        for (r, c), wu_needed in plants_data:
            total_wu_needed += wu_needed
            
        if total_wu_needed == 0:
            return 0
            
        # Cost = (Min LOAD ops) + (Min POUR ops)
        # This is an admissible lower bound because it ignores all movement costs.
        return total_wu_needed * 2