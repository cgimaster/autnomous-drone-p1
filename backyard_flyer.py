import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5

class Constants:
    SQUARE_SIZE = 5.0
    H_SQUARE_SIZE = SQUARE_SIZE / 2.0
    TARGET_TAKEOFF_ALTITUDE = 10.0
    PRECISION = 0.01

class BackyardFlyer(Drone):
    home_position = np.array([0.0, 0.0, 0.0])
    target_takeoff_position = np.array([home_position[0], home_position[1], Constants.TARGET_TAKEOFF_ALTITUDE])
    all_waypoints = []
    next_waypoint_index = 0

    def __init__(self, connection):
        super().__init__(connection)
        self.all_waypoints = self.calculate_box()
        self.in_mission = True
        #self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        # print('Local position {} . '.format(self.local_position), )
        if self.flight_state == States.TAKEOFF:
            altitude = -1.0 * self.local_position[2]
            if altitude > 0.95 * self.target_takeoff_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            next_waypoint = self.all_waypoints[self.next_waypoint_index]
            waypoint_reached = True
            inverted_axis = [1.0,1.0,-1.0]
            for idx in range(0,3):
                if next_waypoint[idx] == 0.0:
                    if self.local_position[idx] > Constants.PRECISION:
                        waypoint_reached = False
                        break
                elif abs(inverted_axis[idx] * self.local_position[idx] - next_waypoint[idx]) / next_waypoint[idx] > Constants.PRECISION:
                    #print('Waypoint not reached {} vs {} at {}'.format(next_waypoint, self.local_position, idx))
                    waypoint_reached = False
                    break

            if waypoint_reached:
                self.next_waypoint_index += 1
                print('Waypoint reached {} vs {}. Next waypoint {}'.format(next_waypoint, self.local_position, self.next_waypoint_index))
                self.waypoint_transition()



    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if ((self.global_position[2] - self.global_home[2] < 0.1) and abs(self.local_position[2]) < Constants.PRECISION):
                self.disarming_transition()
        #print(self.local_velocity)

    def state_callback(self):
        if not self.in_mission: return
        elif self.flight_state == States.MANUAL: self.arming_transition()
        elif self.flight_state == States.ARMING: self.takeoff_transition()
        elif self.flight_state == States.DISARMING: self.manual_transition()

    def calculate_box(self):
        box = [[0.0, 0.0, Constants.TARGET_TAKEOFF_ALTITUDE],
               [Constants.SQUARE_SIZE, 0.0, Constants.TARGET_TAKEOFF_ALTITUDE],
               [Constants.SQUARE_SIZE, Constants.SQUARE_SIZE, Constants.TARGET_TAKEOFF_ALTITUDE],
               [0.0, Constants.SQUARE_SIZE, Constants.TARGET_TAKEOFF_ALTITUDE],
               [0.0, 0.0, Constants.TARGET_TAKEOFF_ALTITUDE]]
        return box


    def arming_transition(self):
        print("arming transition")
        self.take_control()
        self.arm()
        home_position = self.home_position
        print('Home position: ',home_position)
        self.set_home_position(*self.global_position)
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        self.takeoff(Constants.TARGET_TAKEOFF_ALTITUDE)
        self.flight_state = States.TAKEOFF
        print("takeoff transition")


    def waypoint_transition(self):
        """
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")
        if self.next_waypoint_index < 0: return
        if self.next_waypoint_index >= len(self.all_waypoints):
            self.landing_transition()
            return
        next_waypoint = self.all_waypoints[self.next_waypoint_index]
        print("Next waypoint: ", next_waypoint)
        self.cmd_position(*next_waypoint, 0)
        self.flight_state = States.WAYPOINT


    def landing_transition(self):
        """
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    configurable = False
    if configurable:
        parser = argparse.ArgumentParser()
        parser.add_argument('--port', type=int, default=5760, help='Port number')
        parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
        args = parser.parse_args()

        conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    else:
        conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)

    drone = BackyardFlyer(conn)
    time.sleep(1)
    drone.start()
