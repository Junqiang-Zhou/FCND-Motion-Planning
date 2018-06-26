import argparse
import time
import msgpack
from enum import Enum, auto

import networkx as nx
import numpy as np

from planning_utils import a_star_grid, a_star_graph, heuristic, prune_path
from planning_utils import create_grid, create_grid_and_edges, closest_point

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, planning, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.planning_option = planning

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv') as f:
            row1 = f.readline().strip().replace(',','').split(' ')
        home = {row1[0]: float(row1[1]), row1[2]: float(row1[3])}

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(home['lon0'],  home['lat0'],  0.0)

        # TODO: convert to current local position using global_to_local()
        local_north, local_east, local_down = global_to_local(self.global_position, self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        if self.planning_option == 'GRID':
            print('creating grid...')
            # Define a grid for a particular altitude and safety margin around obstacles
            grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        else:
            print('creating graph...')
            grid, edges, north_offset, east_offset = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # Define starting point on the grid (this is just grid center)
        # TODO: convert start position to current position rather than map center
        grid_start = (int(local_north-north_offset), int(local_east-east_offset))

        # Set goal as some arbitrary position on the grid
        # TODO: adapt to set goal as latitude / longitude position and convert
        goal_global = [-122.40196856, 37.79673623, 0.0]
        goal_north, goal_east, goal_down = global_to_local(goal_global, self.global_home)
        grid_goal = (int(goal_north-north_offset), int(goal_east-east_offset))

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        if self.planning_option == 'GRID':
            # Call A* based on grid search
            path, _ = a_star_grid(grid, heuristic, grid_start, grid_goal)
        else:
            # creating a graph first
            G = nx.Graph()
            for e in edges:
                p1 = e[0]
                p2 = e[1]
                dist = np.linalg.norm(np.array(p2) - np.array(p1))
                G.add_edge(p1, p2, weight=dist)
            # find nearest from the graph nodes
            start_ne_g = closest_point(G, grid_start)
            goal_ne_g = closest_point(G, grid_goal)
            # Call A* based on graph search
            path, _ = a_star_graph(G, heuristic, start_ne_g, goal_ne_g)
            # Append the actual start and goal states to path
            path = [grid_start] + path + [grid_goal]

        # TODO: prune path to minimize number of waypoints
        pruned_path = prune_path(path)

        # Convert path to waypoints(use integer for waypoints)
        waypoints = [[int(p[0] + north_offset), int(p[1] + east_offset), TARGET_ALTITUDE, 0] for p in pruned_path]

        # Set self.waypoints
        self.waypoints = waypoints

        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--planning', type=str, default='GRID', help="planning options: GRID or GRAPH")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(args.planning, conn)
    time.sleep(1)

    drone.start()
