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
    min_trip_cost = utils.infinity
    
    for robot_id, (r, c, load, capacity) in robots_map.items():
        # [Same calculation for min_R_T_P_for_this_robot as in the GBFS code]
        # ... (הקוד המלא כאן לא מוצג שוב לנוחות, אך הוא זהה לחישוב min_total_path_cost)
        
        # for simplicity: assuming the full R->T->P calculation is done and stored in min_total_path_cost

        min_R_T_P_for_this_robot = 0 # (Just placeholder for the sake of presentation)
        # ... (הקוד המלא שהצגת קודם מחושב כאן)
        
        # לאחר חישוב min_R_T_P_for_this_robot:
        min_trip_cost = min(min_trip_cost, min_R_T_P_for_this_robot)


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