import ex1_check
import search
import utils
import math

id = ["No numbers - I'm special!"]





class WateringProblem(search.Problem):
    """This class implements a pressure plate problem"""

    def __init__(self, initial):

        grid_size = initial["Size"]
        walls = frozenset(initial["Walls"])
        taps = frozenset(initial["Taps"].items())
        plants = frozenset(initial["Plants"].items())
        robots = frozenset(initial["Robots"].items())

        hashable_initial_tupple = (grid_size, walls, taps, plants, robots)

        
        """ Constructor only needs the initial state.
        Don't forget to set the goal or implement the goal test"""
        
        search.Problem.__init__(self, hashable_initial_tupple)

    def successor(self, state):
        """ Generates the successor states returns [(action, achieved_states, ...)]"""
        successors = []
    
        # Extracting state data (Retrieving State Components by Index)
        # state = (GridSize, Walls, Taps, Plants, Robots)
        grid_size = state[0]
        walls = state[1]
        taps_data = state[2]     # frozenset of ((r, c), wu_rem)
        plants_data = state[3]   # frozenset of ((r, c), wu_needed)
        robots_data = state[4]   # frozenset of (ID, (r, c, load, capacity))
    
        R, C = grid_size # Grid size (rows, columns)
    
        # Convert frozenset to dictionaries for convenient fast lookup (not mandatory, but very convenient!)
        # Note: we use these dictionaries only for reading, not writing (to maintain immutability).
        taps_map = dict(taps_data)
        plants_map = dict(plants_data)
    
        # Prepare a dictionary of all robots by ID, and a list of all current robot locations for collision checking.
        robots_map = {}
        robot_locations = set()
        for robot_id, details in robots_data:
            r, c, load, capacity = details
            robots_map[robot_id] = details
            robot_locations.add((r, c))
        
        # --- Start iterating over each robot to generate actions ---
        # We'll use robots_map to get the details of each robot.
    
        for robot_id, (r, c, load, capacity) in robots_map.items(): 
        
            # 1. Movement actions
            moves = {
                "UP":    (-1, 0), "DOWN":  (1, 0), 
                "LEFT":  (0, -1), "RIGHT": (0, 1)
            }
        
            for action_name, (dr, dc) in moves.items():
                new_r, new_c = r + dr, c + dc
                new_coord = (new_r, new_c)
                
                # --- Check movement legality conditions ---
                
                # 1. Is the new position within bounds?
                if not (0 <= new_r < R and 0 <= new_c < C):
                    continue
                    
                # 2. Is the new position a wall?
                if new_coord in walls:
                    continue
                    
                # 3. Is the new position occupied by another robot? 
                # (If the robot doesn't move, it's always legal because it can stay in place. But here it's moving).
                # We'll check if there's another robot in the new position.
                is_occupied = False
                for other_id, (other_r, other_c, _, _) in robots_map.items():
                    if other_id != robot_id and (other_r, other_c) == new_coord:
                        is_occupied = True
                        break
                
                if is_occupied:
                    continue
                    
                # --- Creating new state for legal movement ---
                
                # a. Update the current robot
                new_robot_details = (new_r, new_c, load, capacity)
                
                # b. Create updated frozenset of Robots (Immutable!)
                
                # 1. Remove the old entry (the old tuple from the frozenset)
                robot_entry_old = (robot_id, robots_map[robot_id]) # (ID, (r, c, load, capacity))
                
                # 2. Add the new entry
                robot_entry_new = (robot_id, new_robot_details)
                
                # 3. Build the new frozenset. 
                # The cleanest way: extract the old frozenset (index 4), remove the old, add the new.
                new_robots_set = set(robots_data) 
                new_robots_set.remove(robot_entry_old) 
                new_robots_set.add(robot_entry_new)
                
                new_robots_data = frozenset(new_robots_set)
                
                # c. Assemble the next_state tuple
                # Only the robots component changes
                next_state = (grid_size, walls, taps_data, plants_data, new_robots_data)
                action_str = f"{action_name}{{{robot_id}}}"
                successors.append((action_str, next_state))
    
            # 2. LOAD action (loading water) 
            if (r, c) in taps_map: # If the robot is on a tap
                wu_in_tap = taps_map[(r, c)]
                
                # Conditions: there is water in the tap (wu_in_tap > 0) and not at full capacity (load < capacity)
                if wu_in_tap > 0 and load < capacity:
                        
                        # --- Creating new state ---
                        
                        # a. Update the robot (load++)
                        new_load = load + 1
                        new_robot_details = (r, c, new_load, capacity)
                        
                        robot_entry_old = (robot_id, robots_map[robot_id])
                        robot_entry_new = (robot_id, new_robot_details)
                        
                        new_robots_set = set(robots_data)
                        new_robots_set.remove(robot_entry_old)
                        new_robots_set.add(robot_entry_new)
                        new_robots_data = frozenset(new_robots_set)
                        
                        # b. Update the tap (wu_in_tap--)
                        new_wu_in_tap = wu_in_tap - 1
                        
                        new_taps_map = taps_map.copy()
                        new_taps_map[(r, c)] = new_wu_in_tap
                            
                        
                        new_taps_data = frozenset(new_taps_map.items()) # Convert back to frozenset
                        
                        # c. Assemble the next_state
                        next_state = (grid_size, walls, new_taps_data, plants_data, new_robots_data)
                        action_str = f"LOAD{{{robot_id}}}"
                        successors.append((action_str, next_state))

            # 3. POUR action (pouring water)
            if (r, c) in plants_map: # If the robot is on a plant
                wu_needed = plants_map[(r, c)]
                
                # Conditions: the robot is carrying water (load > 0) and the plant still needs water (wu_needed > 0)
                if load > 0 and wu_needed > 0:
                        
                        # --- Creating new state ---
                        
                        # a. Update the robot (load--)
                        new_load = load - 1
                        new_robot_details = (r, c, new_load, capacity)
                        
                        robot_entry_old = (robot_id, robots_map[robot_id])
                        robot_entry_new = (robot_id, new_robot_details)
                        
                        new_robots_set = set(robots_data)
                        new_robots_set.remove(robot_entry_old)
                        new_robots_set.add(robot_entry_new)
                        new_robots_data = frozenset(new_robots_set)
                        
                        # b. Update the plant (wu_needed--)
                        new_wu_needed = wu_needed - 1
                        
                        new_plants_map = plants_map.copy()
                        new_plants_map[(r, c)] = new_wu_needed
                            
                        
                        new_plants_data = frozenset(new_plants_map.items()) # Convert back to frozenset
                        
                        # c. Assemble the next_state
                        next_state = (grid_size, walls, taps_data, new_plants_data, new_robots_data)
                        action_str = f"POUR{{{robot_id}}}"
                        successors.append((action_str, next_state))

        return successors

    def goal_test(self, state):
        """ given a state, checks if this is the goal state, compares to the created goal state returns True/False"""
        # Extracting the plants data (index 3)
        # plants_data: frozenset of ((r, c), wu_needed)
        plants_data = state[3]
        
        # The goal is achieved when all plants have wu_needed <= 0.
        # We iterate through all plant entries.
        
        # Check if there is at least one plant that still requires water (wu_needed > 0)
        for (r, c), wu_needed in plants_data:
            if wu_needed > 0:
                # If a plant is found that still needs water, the goal is not achieved
                return False
                
        # If we iterate through all plants and all wu_needed values are 0 (or the set is empty), the goal is achieved
        return True

    def h_astar(self, node):
        """
        Improved Admissible Heuristic for A*.
        Decomposes cost into:
        1. Base Actions (LOAD/POUR)
        2. Initial Move (Robot -> Resource)
        3. Cargo Transport (Tap -> Plant for all required trips)
        4. Commute Returns (Plant -> Tap for multiple trips)
        """
        state = node.state
        grid_size, walls, taps_data, plants_data, robots_data = state
        
        # --- 1. Base Action Cost ---
        total_wu_needed = sum(wu for (coord, wu) in plants_data)
        total_wu_carried = sum(load for (id, (r, c, load, capacity)) in robots_data)
        wu_net_needed = max(0, total_wu_needed - total_wu_carried)
        
        h_base = total_wu_needed + wu_net_needed 
        
        if total_wu_needed == 0:
            return 0

        # Identify Locations
        available_taps = [coord for (coord, wu) in taps_data if wu > 0]
        needed_plants = [coord for (coord, wu) in plants_data if wu > 0]
        
        if not needed_plants: return h_base
        
        # Calculate global minimum distance between any Tap and any Plant (The "Bridge")
        # This is the absolute shortest path water can travel.
        min_TP_dist = utils.infinity
        if available_taps:
            for t in available_taps:
                for p in needed_plants:
                    dist = abs(t[0] - p[0]) + abs(t[1] - p[1])
                    if dist < min_TP_dist:
                        min_TP_dist = dist
        
        # If we need water but have no taps, we can't estimate movement (assume 0 to be safe/admissible)
        if min_TP_dist == utils.infinity: 
            min_TP_dist = 0

        # --- 2. H_Start (Min distance to start working) ---
        # Find the minimum distance ANY robot needs to travel to do a useful action.
        min_start_dist = utils.infinity
        
        for robot_id, (r, c, load, capacity) in robots_data:
            # Option A: Go Pour (if has load)
            if load > 0:
                for p in needed_plants:
                    dist = abs(r - p[0]) + abs(c - p[1])
                    if dist < min_start_dist:
                        min_start_dist = dist
            
            # Option B: Go Load (if needs load)
            if wu_net_needed > 0 and available_taps and load < capacity:
                for t in available_taps:
                    dist = abs(r - t[0]) + abs(c - t[1])
                    if dist < min_start_dist:
                        min_start_dist = dist
                        
        if min_start_dist == utils.infinity:
            min_start_dist = 0

        # --- 3. H_Cargo (Transporting the water) ---
        # How many full trips (Tap -> Plant) are absolutely necessary?
        # We use the max capacity of any robot to be optimistic (admissible).
        max_capacity = 0
        for id, (r, c, load, capacity) in robots_data:
            max_capacity = max(max_capacity, capacity)
            
        min_trips = 0
        if max_capacity > 0 and wu_net_needed > 0:
            min_trips = math.ceil(wu_net_needed / max_capacity)
            
        h_cargo = min_trips * min_TP_dist

        # --- 4. H_Return (Commuting back for next load) ---
        # If we need more than 1 trip, we must drive back from Plant to Tap at least (Trips-1) times.
        h_return = 0
        if min_trips > 1:
            h_return = (min_trips - 1) * min_TP_dist

        # --- Final Sum ---
        # Note: If we only have carried water (min_trips=0), H_Cargo is 0, which is correct.
        # The cost is just H_Base + H_Start (Robot -> Plant).
        
        return h_base + min_start_dist + h_cargo + h_return
    
    def h_gbfs(self, node):
        """
        Improved Admissible Heuristic for A*.
        Decomposes cost into:
        1. Base Actions (LOAD/POUR)
        2. Initial Move (Robot -> Resource)
        3. Cargo Transport (Tap -> Plant for all required trips)
        4. Commute Returns (Plant -> Tap for multiple trips)
        """
        state = node.state
        grid_size, walls, taps_data, plants_data, robots_data = state
        
        # --- 1. Base Action Cost ---
        total_wu_needed = sum(wu for (coord, wu) in plants_data)
        total_wu_carried = sum(load for (id, (r, c, load, capacity)) in robots_data)
        wu_net_needed = max(0, total_wu_needed - total_wu_carried)
        
        h_base = total_wu_needed + wu_net_needed 
        
        if total_wu_needed == 0:
            return 0

        # Identify Locations
        available_taps = [coord for (coord, wu) in taps_data if wu > 0]
        needed_plants = [coord for (coord, wu) in plants_data if wu > 0]
        
        if not needed_plants: return h_base
        
        # Calculate global minimum distance between any Tap and any Plant (The "Bridge")
        # This is the absolute shortest path water can travel.
        min_TP_dist = utils.infinity
        if available_taps:
            for t in available_taps:
                for p in needed_plants:
                    dist = abs(t[0] - p[0]) + abs(t[1] - p[1])
                    if dist < min_TP_dist:
                        min_TP_dist = dist
        
        # If we need water but have no taps, we can't estimate movement (assume 0 to be safe/admissible)
        if min_TP_dist == utils.infinity: 
            min_TP_dist = 0

        # --- 2. H_Start (Min distance to start working) ---
        # Find the minimum distance ANY robot needs to travel to do a useful action.
        min_start_dist = utils.infinity
        
        for robot_id, (r, c, load, capacity) in robots_data:
            # Option A: Go Pour (if has load)
            if load > 0:
                for p in needed_plants:
                    dist = abs(r - p[0]) + abs(c - p[1])
                    if dist < min_start_dist:
                        min_start_dist = dist
            
            # Option B: Go Load (if needs load)
            if wu_net_needed > 0 and available_taps and load < capacity:
                for t in available_taps:
                    dist = abs(r - t[0]) + abs(c - t[1])
                    if dist < min_start_dist:
                        min_start_dist = dist
                        
        if min_start_dist == utils.infinity:
            min_start_dist = 0

        # --- 3. H_Cargo (Transporting the water) ---
        # How many full trips (Tap -> Plant) are absolutely necessary?
        # We use the max capacity of any robot to be optimistic (admissible).
        max_capacity = 0
        for id, (r, c, load, capacity) in robots_data:
            max_capacity = max(max_capacity, capacity)
            
        min_trips = 0
        if max_capacity > 0 and wu_net_needed > 0:
            min_trips = math.ceil(wu_net_needed / max_capacity)
            
        h_cargo = min_trips * min_TP_dist

        # --- 4. H_Return (Commuting back for next load) ---
        # If we need more than 1 trip, we must drive back from Plant to Tap at least (Trips-1) times.
        h_return = 0
        if min_trips > 1:
            h_return = (min_trips - 1) * min_TP_dist

        # --- Final Sum ---
        # Note: If we only have carried water (min_trips=0), H_Cargo is 0, which is correct.
        # The cost is just H_Base + H_Start (Robot -> Plant).
        
        return h_base + min_start_dist + h_cargo + h_return

    # def h_gbfs(self, node):
    #     """
    #     Improved Greedy Heuristic.
    #     Logic:
    #     1. Calculate 'Real Debt': (Water needed by plants) - (Water currently carried).
    #     2. High Penalty for Debt: Multiply Real Debt by a large factor (e.g., 100) to prioritize functional actions (LOAD/POUR).
    #     3. Distance Gradient: Sum the distances of ALL robots to their immediate targets to guide movement.
    #     """
    #     state = node.state
    #     # state = (GridSize, Walls, Taps, Plants, Robots)
    #     taps_data = state[2]
    #     plants_data = state[3]
    #     robots_data = state[4]
        
    #     # --- 1. Calculate Real Debt (התיקון לשאלה 1) ---
    #     total_wu_needed_by_plants = sum(wu for (coord, wu) in plants_data)
    #     total_wu_carried_by_robots = sum(load for (id, (r, c, load, capacity)) in robots_data)
        
    #     # המים שבאמת חסרים במערכת (שעדיין לא הוטענו על רובוט)
    #     # אנחנו לוקחים מקסימום עם 0 למקרה קצה שבו רובוטים מחזיקים יותר ממה שצריך
    #     real_water_debt = max(0, total_wu_needed_by_plants - total_wu_carried_by_robots)
        
    #     if total_wu_needed_by_plants == 0:
    #         return 0
        
    #     # --- 2. Hierarchy Weighting (התשובה לשאלה 2) ---
    #     # אנחנו נותנים משקל עצום לחוב המים כדי שהאלגוריתם תמיד יעדיף להוריד את החוב
    #     # על פני קיצור מרחקים.
    #     h_score = real_water_debt * 100 
        
    #     # --- 3. Guidance for Robots (All Robots Matter) ---
        
    #     # נכין רשימות יעדים
    #     available_taps = [coord for (coord, wu) in taps_data if wu > 0]
    #     needed_plants = [coord for (coord, wu) in plants_data if wu > 0]
        
    #     # מקרה קצה: נתקענו (צריך מים ואין ברזים, או יש מים ואין צמחים)
    #     if (real_water_debt > 0 and not available_taps) or (total_wu_carried_by_robots > 0 and not needed_plants):
    #         return utils.infinity

    #     # לולאה על כל הרובוטים - כל רובוט תורם לציון
    #     for id, (r, c, load, capacity) in robots_data:
    #         robot_coord = (r, c)
    #         dist_to_target = 0
            
    #         # Strategy: Has Water? -> Go Pour. No Water? -> Go Load. (התשובה לשאלה 3)
            
    #         if load > 0 and needed_plants:
    #             # הרובוט מחזיק מים - היעד הוא הצמח הכי קרוב
    #             dist_to_target = min(
    #                 (abs(r - p[0]) + abs(c - p[1])) 
    #                 for p in needed_plants
    #             )
            
    #         elif load == 0 and available_taps:
    #             # הרובוט ריק - היעד הוא הברז הכי קרוב
    #             dist_to_target = min(
    #                 (abs(r - t[0]) + abs(c - t[1])) 
    #                 for t in available_taps
    #             )
            
    #         # מוסיפים את המרחק לציון הכללי.
    #         # ככל שהרובוטים מתקרבים ליעדים שלהם, הציון יורד.
    #         h_score += dist_to_target

    #     return h_score


def create_watering_problem(game):
    print("<<create_watering_problem")
    """ Create a pressure plate problem, based on the description.
    game - tuple of tuples as described in pdf file"""
    return WateringProblem(game)


if __name__ == '__main__':
    ex1_check.main()
