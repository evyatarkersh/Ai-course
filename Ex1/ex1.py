import ex1_check
import search
import utils
from collections import deque

id = ["207456286"]


# MOST UPDATED!!!! BFS ONLY FOR PLANTS AND TAPS!!!!!
class WateringProblem(search.Problem):
    """This class implements a pressure plate problem"""

    def __init__(self, initial):
        # 1. Static Data Handling (Saved in self, removed from State)
        self.grid_size = initial["Size"]
        self.rows, self.cols = self.grid_size
        self.walls = frozenset(initial["Walls"])

        # NEW CODE!!!!!!!!!!!!!!!!
        # המרת הברזים והצמחים לקואורדינטות
        taps_coords = [t[0] for t in initial["Taps"].items()]
        plants_coords = [p[0] for p in initial["Plants"].items()]
        # זהו! רק הם מעניינים
        interesting_points = set(taps_coords + plants_coords)
        # חישוב מרחקים אופטימלי - רק מהיעדים הסטטיים
        self.dists = self._compute_distances_from_sources(interesting_points)
        # END NEW CODE!!!!!!!!!!!!!!!!

        # 2. Pre-calculate Real Distances (BFS)
        # This runs ONCE. We calculate distance from every valid cell to every other valid cell.
        # Structure: self.dists[(r1,c1)][(r2,c2)] = distance_int
        # NEW: PUT LINE 32 IN COMMENT!!!!!!!!!!!!!
        # self.dists = self._compute_all_distances()
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        # 3. Optimize Initial State Structure
        # We use sorted tuples instead of frozensets for faster hashing and iteration.
        # Taps: sorted tuple of ((r, c), amount)
        taps = tuple(sorted(initial["Taps"].items()))
        # Plants: sorted tuple of ((r, c), needed)
        plants = tuple(sorted(initial["Plants"].items()))
        # Robots: sorted tuple of (ID, r, c, load, cap) - flattened for speed
        # Input is {id: (r, c, load, cap)}. We convert to tuple (id, r, c, load, cap)
        robots_list = []
        for r_id, val in initial["Robots"].items():
            robots_list.append((r_id, val[0], val[1], val[2], val[3]))
        robots = tuple(sorted(robots_list))  # Sort by ID

        # --- Pre-calculate Global Min Distance (The "Bridge") ---
        # חישוב חד פעמי של המרחק הכי קצר שיש במפה בין ברז כלשהו לצמח כלשהו
        # זה חוסך לולאות כבדות בתוך ה-Heuristic
        self.min_global_bridge_dist = utils.infinity

        tap_coords = [t[0] for t in taps]
        plant_coords = [p[0] for p in plants]

        if tap_coords and plant_coords:
            for t in tap_coords:
                for p in plant_coords:
                    # שימוש במרחקים האמיתיים שכבר חישבנו (self.dists)
                    if t in self.dists and p in self.dists[t]:
                        d = self.dists[t][p]
                        if d < self.min_global_bridge_dist:
                            self.min_global_bridge_dist = d

        if self.min_global_bridge_dist == utils.infinity:
            self.min_global_bridge_dist = 0

        # The state only contains the dynamic parts
        hashable_initial_state = (taps, plants, robots)

        search.Problem.__init__(self, hashable_initial_state)

    def _compute_distances_from_sources(self, sources):
        """
        Runs BFS only from specific 'source' points (Plants, Taps).
        Returns: distances[source_coord][any_grid_cell] = distance
        """
        distances = {}
        moves = [(0, 1), (0, -1), (1, 0), (-1, 0)]

        for start in sources:
            if start not in distances:
                distances[start] = {}

            # BFS רגיל
            queue = deque([(start, 0)])
            visited = {start}
            distances[start][start] = 0

            while queue:
                (curr_r, curr_c), dist = queue.popleft()

                for dr, dc in moves:
                    nr, nc = curr_r + dr, curr_c + dc
                    neighbor = (nr, nc)

                    if 0 <= nr < self.rows and 0 <= nc < self.cols:
                        if neighbor not in self.walls and neighbor not in visited:
                            visited.add(neighbor)
                            distances[start][neighbor] = dist + 1
                            queue.append((neighbor, dist + 1))
        return distances

    def _compute_all_distances(self):
        """
        Runs BFS from every valid cell to map true distances on the grid.
        Returns a nested dictionary: d[start_cell][end_cell] = distance
        """
        distances = {}
        valid_cells = [(r, c) for r in range(self.rows) for c in range(self.cols)
                       if (r, c) not in self.walls]

        # Optimization: To avoid N^2 full BFS runs if the grid is huge,
        # usually assignments are small enough (e.g. 10x10 or 20x20).
        # We run a BFS starting from 'start' to fill the map.

        moves = [(0, 1), (0, -1), (1, 0), (-1, 0)]

        for start in valid_cells:
            distances[start] = {}
            # Standard BFS
            queue = deque([(start, 0)])
            visited = {start}
            distances[start][start] = 0

            while queue:
                (curr_r, curr_c), dist = queue.popleft()

                for dr, dc in moves:
                    nr, nc = curr_r + dr, curr_c + dc
                    neighbor = (nr, nc)

                    if 0 <= nr < self.rows and 0 <= nc < self.cols:
                        if neighbor not in self.walls and neighbor not in visited:
                            visited.add(neighbor)
                            distances[start][neighbor] = dist + 1
                            queue.append((neighbor, dist + 1))

        return distances

    def get_distance(self, p1, p2):
        """
        Helper to get pre-calculated distance.
        Checks both directions because 'dists' keys are only Taps/Plants.
        """
        # בדיקה רגילה (אם p1 הוא צמח/ברז)
        if p1 in self.dists and p2 in self.dists[p1]:
            return self.dists[p1][p2]

        # בדיקה הפוכה (אם p2 הוא צמח/ברז - וזה המקרה הנפוץ ב-GBFS)
        if p2 in self.dists and p1 in self.dists[p2]:
            return self.dists[p2][p1]

        return utils.infinity

    def successor(self, state):
        """ Generates the successor states. Optimized for speed using Tuples + Single Robot Pruning. """
        successors = []

        # Unpack State
        taps_tuple, plants_tuple, robots_tuple = state

        # 1. Create Lookup Maps (Using existing logic)
        robot_locations = set()
        for r_data in robots_tuple:
            # r_data = (id, r, c, load, cap)
            loc = (r_data[1], r_data[2])
            robot_locations.add(loc)

        taps_map = dict(taps_tuple)  # (r,c) -> amount
        plants_map = dict(plants_tuple)  # (r,c) -> needed

        # --- PRUNING: Single Robot Optimization ---
        # תוספת: אם יש רובוט אחד, בודקים אם אפשר לבצע פעולה מיידית ולסיים
        if len(robots_tuple) == 1:
            # שליפת הנתונים לפי המבנה שלך: (ID, r, c, load, cap)
            r_id, r, c, load, cap = robots_tuple[0]
            curr_loc = (r, c)

            # 1. Pruning Rule: Immediate Pour
            if curr_loc in plants_map:
                needed = plants_map[curr_loc]
                if needed > 0 and load > 0:
                    # יצירת סטייט חדש וחזרה מיידית
                    new_robot = (r_id, r, c, load - 1, cap)
                    new_robots_tuple = (new_robot,)  # טאפל יחיד

                    # עדכון הצמחים (באותו אופן שהקוד המקורי עושה)
                    new_plants_list = list(plants_tuple)
                    for idx, p in enumerate(new_plants_list):
                        if p[0] == curr_loc:
                            new_plants_list[idx] = (p[0], needed - 1)
                            break
                    new_plants_tuple = tuple(new_plants_list)

                    # מחזירים ויוצאים!
                    return [(f"POUR{{{r_id}}}", (taps_tuple, new_plants_tuple, new_robots_tuple))]

            # 2. Pruning Rule: Immediate Load
            if curr_loc in taps_map:
                amount = taps_map[curr_loc]

                # חישוב סך המים שצריך במשחק (כדי לא לטעון סתם אם לא צריך)
                # סוכמים את האיבר השני בכל טאפל ברשימת הצמחים
                total_needed = sum(p[1] for p in plants_tuple)

                # התנאי: יש מים בברז + יש מקום ברובוט + המטען הנוכחי קטן ממה שצריך סה"כ
                if amount > 0 and load < cap and load < total_needed:
                    # יצירת סטייט חדש וחזרה מיידית
                    new_robot = (r_id, r, c, load + 1, cap)
                    new_robots_tuple = (new_robot,)

                    # עדכון הברזים
                    new_taps_list = list(taps_tuple)
                    for idx, t in enumerate(new_taps_list):
                        if t[0] == curr_loc:
                            new_taps_list[idx] = (t[0], amount - 1)
                            break
                    new_taps_tuple = tuple(new_taps_list)

                    # מחזירים ויוצאים!
                    return [(f"LOAD{{{r_id}}}", (new_taps_tuple, plants_tuple, new_robots_tuple))]
        # --- END PRUNING ---

        # המשך הקוד הרגיל שלך...

        # Static moves
        directions = {
            "UP": (-1, 0), "DOWN": (1, 0), "LEFT": (0, -1), "RIGHT": (0, 1)
        }

        # Iterate over robots using the tuple
        for i, r_data in enumerate(robots_tuple):
            robot_id, r, c, load, cap = r_data

            # --- Move Actions ---
            for action_name, (dr, dc) in directions.items():
                new_r, new_c = r + dr, c + dc
                new_coord = (new_r, new_c)

                # Check Bounds
                if not (0 <= new_r < self.rows and 0 <= new_c < self.cols):
                    continue
                # Check Walls (O(1))
                if new_coord in self.walls:
                    continue
                # Check Robot Collision (O(1))
                if new_coord in robot_locations:
                    continue

                # Construct new state
                new_robot_data = (robot_id, new_r, new_c, load, cap)
                new_robots_tuple = robots_tuple[:i] + (new_robot_data,) + robots_tuple[i + 1:]

                next_state = (taps_tuple, plants_tuple, new_robots_tuple)
                successors.append((f"{action_name}{{{robot_id}}}", next_state))

            # --- LOAD Action ---
            if (r, c) in taps_map:
                wu_in_tap = taps_map[(r, c)]
                if wu_in_tap > 0 and load < cap:
                    new_robot_data = (robot_id, r, c, load + 1, cap)
                    new_robots_tuple = robots_tuple[:i] + (new_robot_data,) + robots_tuple[i + 1:]

                    new_wu = wu_in_tap - 1
                    new_taps_list = list(taps_tuple)
                    for idx, t in enumerate(new_taps_list):
                        if t[0] == (r, c):
                            new_taps_list[idx] = (t[0], new_wu)
                            break
                    new_taps_tuple = tuple(new_taps_list)

                    next_state = (new_taps_tuple, plants_tuple, new_robots_tuple)
                    successors.append((f"LOAD{{{robot_id}}}", next_state))

            # --- POUR Action ---
            if (r, c) in plants_map:
                wu_needed = plants_map[(r, c)]
                if load > 0 and wu_needed > 0:
                    new_robot_data = (robot_id, r, c, load - 1, cap)
                    new_robots_tuple = robots_tuple[:i] + (new_robot_data,) + robots_tuple[i + 1:]

                    new_needed = wu_needed - 1
                    new_plants_list = list(plants_tuple)
                    for idx, p in enumerate(new_plants_list):
                        if p[0] == (r, c):
                            new_plants_list[idx] = (p[0], new_needed)
                            break
                    new_plants_tuple = tuple(new_plants_list)

                    next_state = (taps_tuple, new_plants_tuple, new_robots_tuple)
                    successors.append((f"POUR{{{robot_id}}}", next_state))

        return successors

    def goal_test(self, state):
        """
        Checks if all plants have 0 needed water.
        State structure: (Taps, Plants, Robots)
        """
        plants_tuple = state[1]
        for _, wu_needed in plants_tuple:
            if wu_needed > 0:
                return False
        return True

    def h_astar(self, node):
        """
        Final Optimized Admissible Heuristic using Direct Access & Loop Fusion.
        """
        state = node.state
        taps_tuple, plants_tuple, robots_tuple = state

        # 1. Gather Needs (לולאה מהירה על הצמחים)
        total_wu_needed = 0
        needed_plants_coords = []
        for coord, wu in plants_tuple:
            if wu > 0:
                total_wu_needed += wu
                needed_plants_coords.append(coord)

        if total_wu_needed == 0:
            return 0

        # 2. Gather Robot Stats & Calc H_Start (איחוד לולאות!)
        # במקום לרוץ פעמיים על הרובוטים, אנחנו עושים הכל במכה אחת.

        total_wu_carried = 0
        max_capacity = 0
        min_start_dist = utils.infinity

        # שמירת רפרנס למילון המרחקים לגישה מהירה (Speed Optimization)
        dists_map = self.dists
        available_taps = [t[0] for t in taps_tuple if t[1] > 0]

        for _, r_r, r_c, load, capacity in robots_tuple:
            # --- חלק א: חישוב סטטיסטיקה ---
            total_wu_carried += load
            if capacity > max_capacity: max_capacity = capacity

            # --- חלק ב: חישוב H_Start (באותה לולאה) ---
            robot_loc = (r_r, r_c)

            # בדיקת בטיחות: אם הרובוט על קיר (לא אמור לקרות), מדלגים
            # if robot_loc not in dists_map: continue

            # שליפת מילון המרחקים של הרובוט הספציפי הזה ב-O(1)
            # r_dists = dists_map[robot_loc]

            # Option A: Go Pour (הולך לשפוך)
            if load > 0:
                for p_coord in needed_plants_coords:
                    # גישה ישירה למילון במקום פונקציה get_distance
                    if p_coord in dists_map and robot_loc in dists_map[p_coord]:
                        d = dists_map[p_coord][robot_loc]
                        if d < min_start_dist: min_start_dist = d

            # Option B: Go Load (הולך לטעון)
            # תיקון קריטי: מחקנו את התנאי wu_net_needed > 0 כדי לשמור על אדמיסביליות
            if available_taps and load < capacity:
                for t in available_taps:
                    if t in dists_map and robot_loc in dists_map[t]:
                        d = dists_map[t][robot_loc]
                        if d < min_start_dist: min_start_dist = d

        if min_start_dist == utils.infinity: min_start_dist = 0

        # 3. H_Cargo (חישוב מתמטי ב-O(1) וללא לולאות)
        wu_net_needed = max(0, total_wu_needed - total_wu_carried)
        h_base = total_wu_needed + wu_net_needed

        h_cargo = 0
        if max_capacity > 0 and wu_net_needed > 0:
            # חישוב עיגול כלפי מעלה מהיר (Integer Ceiling Division)
            min_trips = (wu_net_needed + max_capacity - 1) // max_capacity
            # שימוש בערך שחישבנו מראש ב-Init
            h_cargo = min_trips * self.min_global_bridge_dist

        return h_base + min_start_dist + h_cargo

    def h_gbfs(self, node):
        """
        Improved Greedy Heuristic using TRUE GRID DISTANCES (BFS).
        Logic preserved: Real Debt * 100 + Distances
        """
        state = node.state
        taps_tuple, plants_tuple, robots_tuple = state

        # --- 1. Calculate Real Debt ---
        total_wu_needed_by_plants = sum(wu for (_, wu) in plants_tuple)
        total_wu_carried_by_robots = sum(load for (_, _, _, load, _) in robots_tuple)

        real_water_debt = max(0, total_wu_needed_by_plants - total_wu_carried_by_robots)

        if total_wu_needed_by_plants == 0:
            return 0

        # --- 2. Hierarchy Weighting ---
        h_score = real_water_debt * 100

        # --- 3. Guidance for Robots ---
        available_taps = [coord for (coord, wu) in taps_tuple if wu > 0]
        needed_plants = [coord for (coord, wu) in plants_tuple if wu > 0]

        # Edge cases check
        if (real_water_debt > 0 and not available_taps):
            return utils.infinity

        for _, r, c, load, capacity in robots_tuple:
            robot_coord = (r, c)
            dist_to_target = 0

            # Strategy: Has Water? -> Go Pour. No Water? -> Go Load.
            if load > 0 and needed_plants:
                dist_to_target = utils.infinity
                for p in needed_plants:
                    # TRUE DISTANCE
                    d = self.get_distance(robot_coord, p)
                    if d < dist_to_target:
                        dist_to_target = d

            elif load == 0 and available_taps:
                dist_to_target = utils.infinity
                for t in available_taps:
                    # TRUE DISTANCE
                    d = self.get_distance(robot_coord, t)
                    if d < dist_to_target:
                        dist_to_target = d

            # If unreachable (infinity), we keep it huge to discourage this state
            h_score += dist_to_target

        return h_score


def create_watering_problem(game):
    print("<<create_watering_problem")
    """ Create a pressure plate problem, based on the description.
        game - tuple of tuples as described in pdf file"""
    return WateringProblem(game)


if __name__ == '__main__':
    ex1_check.main()
