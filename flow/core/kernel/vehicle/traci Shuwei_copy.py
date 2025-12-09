"""Script containing the TraCI vehicle kernel class."""
import traceback

from flow.core.kernel.vehicle import KernelVehicle
import traci.constants as tc
from traci.exceptions import FatalTraCIError, TraCIException
import numpy as np
import collections
import warnings
from flow.controllers.car_following_models import SimCarFollowingController
from flow.controllers.rlcontroller import RLController
from flow.controllers.lane_change_controllers import SimLaneChangeController
from bisect import bisect_left
import itertools
from copy import deepcopy

# colors for vehicles
WHITE = (255, 255, 255)
CYAN = (0, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
STEPS = 10
rdelta = 255 / STEPS
# smoothly go from red to green as the speed increases
color_bins = [[int(255 - rdelta * i), int(rdelta * i), 0] for i in
              range(STEPS + 1)]


class TraCIVehicle(KernelVehicle):
    """Flow kernel for the TraCI API.

    Extends flow.core.kernel.vehicle.base.KernelVehicle
    """

    def __init__(self,
                 master_kernel,
                 sim_params):
        """See parent class."""
        KernelVehicle.__init__(self, master_kernel, sim_params)

        self.__ids = []  # ids of all vehicles
        self.__human_ids = []  # ids of human-driven vehicles
        self.__controlled_ids = []  # ids of flow-controlled vehicles
        self.__controlled_lc_ids = []  # ids of flow lc-controlled vehicles
        self.__rl_ids = []  # ids of rl-controlled vehicles
        self.__observed_ids = []  # ids of the observed vehicles

        # vehicles: Key = Vehicle ID, Value = Dictionary describing the vehicle
        # Ordered dictionary used to keep neural net inputs in order
        self.__vehicles = collections.OrderedDict()

        # create a sumo_observations variable that will carry all information
        # on the state of the vehicles for a given time step
        self.__sumo_obs = {}

        # total number of vehicles in the network
        self.num_vehicles = 0
        # number of rl vehicles in the network
        self.num_rl_vehicles = 0
        # number of vehicles  loaded but not departed vehicles
        self.num_not_departed = 0

        # contains the parameters associated with each type of vehicle
        self.type_parameters = {}

        # contain the minGap attribute of each type of vehicle
        self.minGap = {}

        # list of vehicle ids located in each edge in the network
        self._ids_by_edge = dict()

        # number of vehicles that entered the network for every time-step
        self._num_departed = []
        self._departed_ids = 0

        # number of vehicles to exit the network for every time-step
        self._num_arrived = []
        self._arrived_ids = 0
        self._arrived_rl_ids = []

        # whether or not to automatically color vehicles
        try:
            self._color_by_speed = sim_params.color_by_speed
            self._force_color_update = sim_params.force_color_update
        except AttributeError:
            self._force_color_update = False

        # old speeds used to compute accelerations
        self.previous_speeds = {}

    def initialize(self, vehicles):
        """Initialize vehicle state information.

        This is responsible for collecting vehicle type information from the
        VehicleParams object and placing them within the Vehicles kernel.

        Parameters
        ----------
        vehicles : flow.core.params.VehicleParams
            initial vehicle parameter information, including the types of
            individual vehicles and their initial speeds
        """
        self.type_parameters = vehicles.type_parameters
        self.minGap = vehicles.minGap
        self.num_vehicles = 0
        self.num_rl_vehicles = 0
        self.num_not_departed = 0

        self.__vehicles.clear()
        for typ in vehicles.initial:
            for i in range(typ['num_vehicles']):
                veh_id = '{}_{}'.format(typ['veh_id'], i)
                self.__vehicles[veh_id] = dict()
                self.__vehicles[veh_id]['type'] = typ['veh_id']
                self.__vehicles[veh_id]['initial_speed'] = typ['initial_speed']
                self.num_vehicles += 1
                if typ['acceleration_controller'][0] == RLController:
                    self.num_rl_vehicles += 1

    def update(self, reset):
        """Update vehicle states in the TraCI kernel.

        Ensures lane_leaders is initialized for all vehicles, including during reset,
        new vehicle arrivals, and template vehicle additions.
        """
        # 获取所有车辆 ID 和道路车道数
        vehicle_ids = self.kernel_api.vehicle.getIDList()
        num_lanes = 1  # 默认车道数
        if vehicle_ids:
            try:
                road_id = self.kernel_api.vehicle.getRoadID(vehicle_ids[0])
                edge_lanes = self.kernel_api.lane.getIDList()
                num_lanes = len([lane_id for lane_id in edge_lanes if lane_id.startswith(road_id + "_")])
                # num_lanes = self.kernel_api.lane.getLaneNumber(road_id)
            except Exception as e:
                print(f"Error getting lane number: {e}")

        # 初始化 lane_leaders 为默认值
        for veh_id in vehicle_ids:
            if veh_id not in self.__vehicles:
                self.__vehicles[veh_id] = {}
            if "lane_leaders" not in self.__vehicles[veh_id]:
                self.__vehicles[veh_id]["lane_leaders"] = [None] * num_lanes
                self.__vehicles[veh_id]["lane_headways"] = [1000.0] * num_lanes
                # print(f"Initialized lane_leaders for {veh_id}: {self.__vehicles[veh_id]['lane_leaders']}")

        # 复制前一步的速度
        vehicle_obs = {}
        for veh_id in self.__ids:
            self.previous_speeds[veh_id] = self.get_speed(veh_id)
            vehicle_obs[veh_id] = self.kernel_api.vehicle.getSubscriptionResults(veh_id)
        sim_obs = self.kernel_api.simulation.getSubscriptionResults()

        arrived_rl_ids = []
        # 移除离开的车辆
        for veh_id in sim_obs[tc.VAR_ARRIVED_VEHICLES_IDS]:
            if veh_id in self.get_rl_ids():
                arrived_rl_ids.append(veh_id)
            if veh_id in sim_obs[tc.VAR_TELEPORT_STARTING_VEHICLES_IDS]:
                vehicle_obs[veh_id] = self.__sumo_obs.get(veh_id, {})
            self.remove(veh_id)
            vehicle_obs.pop(veh_id, None)
        self._arrived_rl_ids.append(arrived_rl_ids)

        # 添加新进入的车辆
        for veh_id in sim_obs[tc.VAR_DEPARTED_VEHICLES_IDS]:
            if veh_id in self.get_ids() and vehicle_obs[veh_id] is not None:
                pass
            else:
                veh_type = self.kernel_api.vehicle.getTypeID(veh_id)
                obs = self._add_departed(veh_id, veh_type)
                vehicle_obs[veh_id] = obs
                # 为新车辆初始化 lane_leaders
                self.__vehicles[veh_id]["lane_leaders"] = [None] * num_lanes
                self.__vehicles[veh_id]["lane_headways"] = [1000.0] * num_lanes
                # print(f"New vehicle {veh_id}: lane_leaders={self.__vehicles[veh_id]['lane_leaders']}")

        if reset:
            self.time_counter = 0
            self.prev_last_lc = {}
            for veh_id in self.__rl_ids:
                self.__vehicles[veh_id]["last_lc"] = -float("inf")
                self.prev_last_lc[veh_id] = -float("inf")
            self._num_departed.clear()
            self._num_arrived.clear()
            self._departed_ids = 0
            self._arrived_ids = 0
            self._arrived_rl_ids.clear()
            self.num_not_departed = 0

            # 处理网络模板中的车辆
            if hasattr(self.master_kernel.network.network, "template_vehicles"):
                for veh_id in self.master_kernel.network.network.template_vehicles:
                    vals = deepcopy(self.master_kernel.network.network.template_vehicles[veh_id])
                    vals['depart'] = str(float(vals['depart']) + 2 * self.sim_step)
                    self.kernel_api.vehicle.addFull(veh_id, f'route{veh_id}_0', **vals)
                    self.__vehicles[veh_id]["lane_leaders"] = [None] * num_lanes
                    self.__vehicles[veh_id]["lane_headways"] = [1000.0] * num_lanes
                    # print(f"Template vehicle {veh_id}: lane_leaders={self.__vehicles[veh_id]['lane_leaders']}")

        else:
            self.time_counter += 1
            for veh_id in self.__rl_ids:
                prev_lane = self.get_lane(veh_id)
                if vehicle_obs[veh_id][tc.VAR_LANE_INDEX] != prev_lane:
                    self.__vehicles[veh_id]["last_lc"] = self.time_counter

            self._num_departed.append(sim_obs[tc.VAR_LOADED_VEHICLES_NUMBER])
            self._num_arrived.append(sim_obs[tc.VAR_ARRIVED_VEHICLES_NUMBER])
            self._departed_ids = sim_obs[tc.VAR_DEPARTED_VEHICLES_IDS]
            self._arrived_ids = sim_obs[tc.VAR_ARRIVED_VEHICLES_IDS]
            self.num_not_departed += sim_obs[tc.VAR_LOADED_VEHICLES_NUMBER] - \
                sim_obs[tc.VAR_DEPARTED_VEHICLES_NUMBER]

        # 更新 headway、leader 和 follower
        for veh_id in self.__ids:
            try:
                _position = vehicle_obs.get(veh_id, {}).get(tc.VAR_POSITION, -1001)
                _angle = vehicle_obs.get(veh_id, {}).get(tc.VAR_ANGLE, -1001)
                _time_step = sim_obs[tc.VAR_TIME_STEP]
                _time_delta = sim_obs[tc.VAR_DELTA_T]
                self.__vehicles[veh_id]["orientation"] = list(_position) + [_angle]
                self.__vehicles[veh_id]["timestep"] = _time_step
                self.__vehicles[veh_id]["timedelta"] = _time_delta
            except TypeError:
                print(traceback.format_exc())
            headway = vehicle_obs.get(veh_id, {}).get(tc.VAR_LEADER, None)
            if headway is None:
                self.__vehicles[veh_id]["leader"] = None
                self.__vehicles[veh_id]["follower"] = None
                self.__vehicles[veh_id]["headway"] = 1e+3
                self.__vehicles[veh_id]["follower_headway"] = 1e+3
            else:
                min_gap = self.minGap[self.get_type(veh_id)]
                self.__vehicles[veh_id]["headway"] = headway[1] + min_gap
                self.__vehicles[veh_id]["leader"] = headway[0]
                if headway[0] in self.__vehicles:
                    leader = self.__vehicles[headway[0]]
                    if ("follower_headway" not in leader or
                            headway[1] + min_gap < leader["follower_headway"]):
                        leader["follower"] = veh_id
                        leader["follower_headway"] = headway[1] + min_gap

        # 更新 sumo 观测
        self.__sumo_obs = vehicle_obs.copy()

        # 更新多车道 headways 和 lane_leaders
        try:
            self._multi_lane_headways()
        except Exception as e:
            print(f"Error in _multi_lane_headways: {e}")
            # 回退到默认值
            for veh_id in self.__ids:
                self.__vehicles[veh_id]["lane_leaders"] = [None] * num_lanes
                self.__vehicles[veh_id]["lane_headways"] = [1000.0] * num_lanes
                print(f"Fallback lane_leaders for {veh_id}: {self.__vehicles[veh_id]['lane_leaders']}")

        # 确保 RL 车辆列表排序
        self.__rl_ids.sort()

    def _add_departed(self, veh_id, veh_type):
        """Add a vehicle that entered the network from an inflow or reset.

        Parameters
        ----------
        veh_id: str
            name of the vehicle
        veh_type: str
            type of vehicle, as specified to sumo

        Returns
        -------
        dict
            subscription results from the new vehicle
        """
        if veh_type not in self.type_parameters:
            raise KeyError("Entering vehicle is not a valid type.")

        if veh_id not in self.__ids:
            self.__ids.append(veh_id)
        if veh_id not in self.__vehicles:
            self.num_vehicles += 1
            self.__vehicles[veh_id] = dict()

        # specify the type
        self.__vehicles[veh_id]["type"] = veh_type

        car_following_params = \
            self.type_parameters[veh_type]["car_following_params"]

        # specify the acceleration controller class
        accel_controller = \
            self.type_parameters[veh_type]["acceleration_controller"]
        self.__vehicles[veh_id]["acc_controller"] = \
            accel_controller[0](veh_id,
                                car_following_params=car_following_params,
                                **accel_controller[1])

        # specify the lane-changing controller class
        lc_controller = \
            self.type_parameters[veh_type]["lane_change_controller"]
        self.__vehicles[veh_id]["lane_changer"] = \
            lc_controller[0](veh_id=veh_id, **lc_controller[1])

        # specify the routing controller class
        rt_controller = self.type_parameters[veh_type]["routing_controller"]
        if rt_controller is not None:
            self.__vehicles[veh_id]["router"] = \
                rt_controller[0](veh_id=veh_id, router_params=rt_controller[1])
        else:
            self.__vehicles[veh_id]["router"] = None

        # add the vehicle's id to the list of vehicle ids
        if accel_controller[0] == RLController:
            if veh_id not in self.__rl_ids:
                self.__rl_ids.append(veh_id)
        else:
            if veh_id not in self.__human_ids:
                self.__human_ids.append(veh_id)
                if accel_controller[0] != SimCarFollowingController:
                    self.__controlled_ids.append(veh_id)
                if lc_controller[0] != SimLaneChangeController:
                    self.__controlled_lc_ids.append(veh_id)

        # subscribe the new vehicle
        self.kernel_api.vehicle.subscribe(veh_id, [
            tc.VAR_LANE_INDEX, tc.VAR_LANEPOSITION,
            tc.VAR_ROAD_ID,
            tc.VAR_SPEED,
            tc.VAR_EDGES,
            tc.VAR_POSITION,
            tc.VAR_ANGLE,
            tc.VAR_SPEED_WITHOUT_TRACI,
            tc.VAR_FUELCONSUMPTION,
            tc.VAR_DISTANCE
        ])
        self.kernel_api.vehicle.subscribeLeader(veh_id, 2000)

        # some constant vehicle parameters to the vehicles class
        self.__vehicles[veh_id]["length"] = self.kernel_api.vehicle.getLength(
            veh_id)

        # set the "last_lc" parameter of the vehicle
        self.__vehicles[veh_id]["last_lc"] = -float("inf")

        # specify the initial speed
        self.__vehicles[veh_id]["initial_speed"] = \
            self.type_parameters[veh_type]["initial_speed"]

        # set the speed mode for the vehicle
        speed_mode = self.type_parameters[veh_type][
            "car_following_params"].speed_mode
        self.kernel_api.vehicle.setSpeedMode(veh_id, speed_mode)

        # set the lane changing mode for the vehicle
        lc_mode = self.type_parameters[veh_type][
            "lane_change_params"].lane_change_mode
        self.kernel_api.vehicle.setLaneChangeMode(veh_id, lc_mode)

        # get initial state info
        self.__sumo_obs[veh_id] = dict()
        self.__sumo_obs[veh_id][tc.VAR_ROAD_ID] = \
            self.kernel_api.vehicle.getRoadID(veh_id)
        self.__sumo_obs[veh_id][tc.VAR_LANEPOSITION] = \
            self.kernel_api.vehicle.getLanePosition(veh_id)
        self.__sumo_obs[veh_id][tc.VAR_LANE_INDEX] = \
            self.kernel_api.vehicle.getLaneIndex(veh_id)
        self.__sumo_obs[veh_id][tc.VAR_SPEED] = \
            self.kernel_api.vehicle.getSpeed(veh_id)
        self.__sumo_obs[veh_id][tc.VAR_FUELCONSUMPTION] = \
            self.kernel_api.vehicle.getFuelConsumption(veh_id)

        # make sure that the order of rl_ids is kept sorted
        self.__rl_ids.sort()
        self.num_rl_vehicles = len(self.__rl_ids)

        # get the subscription results from the new vehicle
        new_obs = self.kernel_api.vehicle.getSubscriptionResults(veh_id)

        return new_obs

    def reset(self):
        """See parent class."""
        self.previous_speeds = {}

    def remove(self, veh_id):
        """See parent class."""
        # remove from sumo
        if veh_id in self.kernel_api.vehicle.getIDList():
            self.kernel_api.vehicle.unsubscribe(veh_id)
            self.kernel_api.vehicle.remove(veh_id)

        if veh_id in self.__ids:
            self.__ids.remove(veh_id)

        # remove from the vehicles kernel
        if veh_id in self.__vehicles:
            del self.__vehicles[veh_id]

        if veh_id in self.__sumo_obs:
            del self.__sumo_obs[veh_id]

        # remove it from all other id lists (if it is there)
        if veh_id in self.__human_ids:
            self.__human_ids.remove(veh_id)
            if veh_id in self.__controlled_ids:
                self.__controlled_ids.remove(veh_id)
            if veh_id in self.__controlled_lc_ids:
                self.__controlled_lc_ids.remove(veh_id)
        elif veh_id in self.__rl_ids:
            self.__rl_ids.remove(veh_id)
            # make sure that the rl ids remain sorted
            self.__rl_ids.sort()

        # modify the number of vehicles and RL vehicles
        self.num_vehicles = len(self.get_ids())
        self.num_rl_vehicles = len(self.get_rl_ids())

    def test_set_speed(self, veh_id, speed):
        """Set the speed of the specified vehicle."""
        self.__sumo_obs[veh_id][tc.VAR_SPEED] = speed

    def test_set_edge(self, veh_id, edge):
        """Set the speed of the specified vehicle."""
        self.__sumo_obs[veh_id][tc.VAR_ROAD_ID] = edge

    def set_follower(self, veh_id, follower):
        """Set the follower of the specified vehicle."""
        self.__vehicles[veh_id]["follower"] = follower

    def set_headway(self, veh_id, headway):
        """Set the headway of the specified vehicle."""
        self.__vehicles[veh_id]["headway"] = headway

    def get_orientation(self, veh_id):
        """See parent class."""
        return self.__vehicles[veh_id]["orientation"]

    def get_timestep(self, veh_id):
        """See parent class."""
        return self.__vehicles[veh_id]["timestep"]

    def get_timedelta(self, veh_id):
        """See parent class."""
        return self.__vehicles[veh_id]["timedelta"]

    def get_type(self, veh_id):
        """Return the type of the vehicle of veh_id."""
        return self.__vehicles[veh_id]["type"]

    def get_initial_speed(self, veh_id):
        """Return the initial speed of the vehicle of veh_id."""
        return self.__vehicles[veh_id]["initial_speed"]

    def get_ids(self):
        """See parent class."""
        return self.__ids

    def get_human_ids(self):
        """See parent class."""
        return self.__human_ids

    def get_controlled_ids(self):
        """See parent class."""
        return self.__controlled_ids

    def get_controlled_lc_ids(self):
        """See parent class."""
        return self.__controlled_lc_ids

    def get_rl_ids(self):
        """See parent class."""
        return self.__rl_ids

    def set_observed(self, veh_id):
        """See parent class."""
        if veh_id not in self.__observed_ids:
            self.__observed_ids.append(veh_id)

    def remove_observed(self, veh_id):
        """See parent class."""
        if veh_id in self.__observed_ids:
            self.__observed_ids.remove(veh_id)

    def get_observed_ids(self):
        """See parent class."""
        return self.__observed_ids

    def get_ids_by_edge(self, edges):
        """See parent class."""
        if isinstance(edges, (list, np.ndarray)):
            return sum([self.get_ids_by_edge(edge) for edge in edges], [])
        return self._ids_by_edge.get(edges, []) or []

    def get_inflow_rate(self, time_span):
        """See parent class."""
        if len(self._num_departed) == 0:
            return 0
        num_inflow = self._num_departed[-int(time_span / self.sim_step):]
        return 3600 * sum(num_inflow) / (len(num_inflow) * self.sim_step)

    def get_outflow_rate(self, time_span):
        """See parent class."""
        if len(self._num_arrived) == 0:
            return 0
        num_outflow = self._num_arrived[-int(time_span / self.sim_step):]
        return 3600 * sum(num_outflow) / (len(num_outflow) * self.sim_step)

    def get_num_arrived(self):
        """See parent class."""
        if len(self._num_arrived) > 0:
            return self._num_arrived[-1]
        else:
            return 0

    def get_arrived_ids(self):
        """See parent class."""
        return self._arrived_ids

    def get_arrived_rl_ids(self, k=1):
        """See parent class."""
        if len(self._arrived_rl_ids) > 0:
            arrived = []
            for arr in self._arrived_rl_ids[-k:]:
                arrived.extend(arr)
            return arrived
        else:
            return 0

    def get_departed_ids(self):
        """See parent class."""
        return self._departed_ids

    def get_num_not_departed(self):
        """See parent class."""
        return self.num_not_departed

    def get_fuel_consumption(self, veh_id, error=-1001):
        """Return fuel consumption in gallons/s."""
        ml_to_gallons = 0.000264172
        if isinstance(veh_id, (list, np.ndarray)):
            return [self.get_fuel_consumption(vehID, error) for vehID in veh_id]
        return self.__sumo_obs.get(veh_id, {}).get(tc.VAR_FUELCONSUMPTION, error) * ml_to_gallons

    def get_previous_speed(self, veh_id, error=-1001):
        """See parent class."""
        if isinstance(veh_id, (list, np.ndarray)):
            return [self.get_previous_speed(vehID, error) for vehID in veh_id]
        return self.previous_speeds.get(veh_id, 0)

    def get_speed(self, veh_id, error=-1001):
        """See parent class."""
        if isinstance(veh_id, (list, np.ndarray)):
            return [self.get_speed(vehID, error) for vehID in veh_id]
        return self.__sumo_obs.get(veh_id, {}).get(tc.VAR_SPEED, error)

    def get_default_speed(self, veh_id, error=-1001):
        """See parent class."""
        if isinstance(veh_id, (list, np.ndarray)):
            return [self.get_default_speed(vehID, error) for vehID in veh_id]
        return self.__sumo_obs.get(veh_id, {}).get(tc.VAR_SPEED_WITHOUT_TRACI,
                                                   error)

    def get_position(self, veh_id, error=-1001):
        """See parent class."""
        if isinstance(veh_id, (list, np.ndarray)):
            return [self.get_position(vehID, error) for vehID in veh_id]
        return self.__sumo_obs.get(veh_id, {}).get(tc.VAR_LANEPOSITION, error)

    def get_edge(self, veh_id, error=""):
        """See parent class."""
        if isinstance(veh_id, (list, np.ndarray)):
            return [self.get_edge(vehID, error) for vehID in veh_id]
        return self.__sumo_obs.get(veh_id, {}).get(tc.VAR_ROAD_ID, error)

    def get_lane(self, veh_id, error=-1001):
        """See parent class."""
        if isinstance(veh_id, (list, np.ndarray)):
            return [self.get_lane(vehID, error) for vehID in veh_id]
        return self.__sumo_obs.get(veh_id, {}).get(tc.VAR_LANE_INDEX, error)

    def get_route(self, veh_id, error=None):
        """See parent class."""
        if error is None:
            error = list()
        if isinstance(veh_id, (list, np.ndarray)):
            return [self.get_route(vehID, error) for vehID in veh_id]
        return self.__sumo_obs.get(veh_id, {}).get(tc.VAR_EDGES, error)

    def get_length(self, veh_id, error=-1001):
        """See parent class."""
        if isinstance(veh_id, (list, np.ndarray)):
            return [self.get_length(vehID, error) for vehID in veh_id]
        return self.__vehicles.get(veh_id, {}).get("length", error)

    def get_leader(self, veh_id, error=""):
        """See parent class."""
        if isinstance(veh_id, (list, np.ndarray)):
            return [self.get_leader(vehID, error) for vehID in veh_id]
        return self.__vehicles.get(veh_id, {}).get("leader", error)

    def get_follower(self, veh_id, error=""):
        """See parent class."""
        if isinstance(veh_id, (list, np.ndarray)):
            return [self.get_follower(vehID, error) for vehID in veh_id]
        return self.__vehicles.get(veh_id, {}).get("follower", error)

    def get_headway(self, veh_id, error=-1001):
        """See parent class."""
        if isinstance(veh_id, (list, np.ndarray)):
            return [self.get_headway(vehID, error) for vehID in veh_id]
        return self.__vehicles.get(veh_id, {}).get("headway", error)

    def get_last_lc(self, veh_id, error=-1001):
        """See parent class."""
        if isinstance(veh_id, (list, np.ndarray)):
            return [self.get_headway(vehID, error) for vehID in veh_id]

        if veh_id not in self.__rl_ids:
            warnings.warn('Vehicle {} is not RL vehicle, "last_lc" term set to'
                          ' {}.'.format(veh_id, error))
            return error
        else:
            return self.__vehicles.get(veh_id, {}).get("headway", error)

    def get_acc_controller(self, veh_id, error=None):
        """See parent class."""
        if isinstance(veh_id, (list, np.ndarray)):
            return [self.get_acc_controller(vehID, error) for vehID in veh_id]
        return self.__vehicles.get(veh_id, {}).get("acc_controller", error)

    def get_lane_changing_controller(self, veh_id, error=None):
        """See parent class."""
        if isinstance(veh_id, (list, np.ndarray)):
            return [
                self.get_lane_changing_controller(vehID, error)
                for vehID in veh_id
            ]
        return self.__vehicles.get(veh_id, {}).get("lane_changer", error)

    def get_routing_controller(self, veh_id, error=None):
        """See parent class."""
        if isinstance(veh_id, (list, np.ndarray)):
            return [
                self.get_routing_controller(vehID, error) for vehID in veh_id
            ]
        return self.__vehicles.get(veh_id, {}).get("router", error)

    def set_lane_headways(self, veh_id, lane_headways):
        """Set the lane headways of the specified vehicle."""
        self.__vehicles[veh_id]["lane_headways"] = lane_headways

    def get_lane_headways(self, veh_id, error=None):
        """See parent class."""
        if error is None:
            error = list()
        if isinstance(veh_id, (list, np.ndarray)):
            return [self.get_lane_headways(vehID, error) for vehID in veh_id]
        return self.__vehicles.get(veh_id, {}).get("lane_headways", error)

    def get_lane_leaders_speed(self, veh_id, error=None):
        """See parent class."""
        lane_leaders = self.get_lane_leaders(veh_id)
        return [0 if lane_leader == '' else self.get_speed(lane_leader)
                for lane_leader in lane_leaders]

    def get_lane_followers_speed(self, veh_id, error=None):
        """See parent class."""
        lane_followers = self.get_lane_followers(veh_id)
        return [0 if lane_follower == '' else self.get_speed(lane_follower)
                for lane_follower in lane_followers]

    def set_lane_leaders(self, veh_id, lane_leaders):
        """Set the lane leaders of the specified vehicle."""
        self.__vehicles[veh_id]["lane_leaders"] = lane_leaders

    def get_lane_leaders(self, veh_id, error=None):
        """See parent class."""
        if error is None:
            error = list()
        if isinstance(veh_id, (list, np.ndarray)):
            return [self.get_lane_leaders(vehID, error) for vehID in veh_id]
        return self.__vehicles[veh_id]["lane_leaders"]

    def set_lane_tailways(self, veh_id, lane_tailways):
        """Set the lane tailways of the specified vehicle."""
        self.__vehicles[veh_id]["lane_tailways"] = lane_tailways

    def get_lane_tailways(self, veh_id, error=None):
        """See parent class."""
        if error is None:
            error = list()
        if isinstance(veh_id, (list, np.ndarray)):
            return [self.get_lane_tailways(vehID, error) for vehID in veh_id]
        return self.__vehicles.get(veh_id, {}).get("lane_tailways", error)

    def set_lane_followers(self, veh_id, lane_followers):
        """Set the lane followers of the specified vehicle."""
        self.__vehicles[veh_id]["lane_followers"] = lane_followers

    def get_lane_followers(self, veh_id, error=None):
        """See parent class."""
        if error is None:
            error = list()
        if isinstance(veh_id, (list, np.ndarray)):
            return [self.get_lane_followers(vehID, error) for vehID in veh_id]
        return self.__vehicles.get(veh_id, {}).get("lane_followers", error)

    def _multi_lane_headways(self):
        """Compute multi-lane data for all vehicles."""
        edge_list = self.master_kernel.network.get_edge_list()
        junction_list = self.master_kernel.network.get_junction_list()
        tot_list = edge_list + junction_list
        max_lanes = max([self.master_kernel.network.num_lanes(edge_id) for edge_id in tot_list], default=1)

        edge_dict = {edge: [[] for _ in range(max_lanes)] for edge in tot_list}

        for veh_id in self.get_ids():
            try:
                edge = self.get_edge(veh_id)
                lane = self.get_lane(veh_id)
                pos = self.get_position(veh_id)
                if edge and edge in edge_dict and lane >= 0 and pos != -1001:
                    edge_dict[edge][lane].append((veh_id, pos))
                else:
                    # print(f"Skipping {veh_id}: invalid edge={edge}, lane={lane}, pos={pos}")
                    num_lanes = self.master_kernel.network.num_lanes(edge) if edge else max_lanes
                    self._TraCIVehicle__vehicles[veh_id]["lane_leaders"] = [None] * num_lanes
                    self._TraCIVehicle__vehicles[veh_id]["lane_headways"] = [1000.0] * num_lanes
                    self._TraCIVehicle__vehicles[veh_id]["lane_followers"] = [None] * num_lanes
                    self._TraCIVehicle__vehicles[veh_id]["lane_tailways"] = [1000.0] * num_lanes
            except Exception as e:
                print(f"Error processing {veh_id} in edge_dict: {e}")
                num_lanes = self.master_kernel.network.num_lanes(edge) if edge else max_lanes
                self._TraCIVehicle__vehicles[veh_id]["lane_leaders"] = [None] * num_lanes
                self._TraCIVehicle__vehicles[veh_id]["lane_headways"] = [1000.0] * num_lanes
                self._TraCIVehicle__vehicles[veh_id]["lane_followers"] = [None] * num_lanes
                self._TraCIVehicle__vehicles[veh_id]["lane_tailways"] = [1000.0] * num_lanes

        for edge in edge_dict:
            for lane in range(max_lanes):
                edge_dict[edge][lane].sort(key=lambda x: x[1])

        for veh_id in self.get_ids():
            try:
                edge = self.get_edge(veh_id)
                if edge and edge in edge_dict:
                    headways, tailways, leaders, followers = \
                        self._multi_lane_headways_util(veh_id, edge_dict, len(tot_list))
                    self.set_lane_headways(veh_id, headways)
                    self.set_lane_tailways(veh_id, tailways)
                    self.set_lane_leaders(veh_id, leaders)
                    self.set_lane_followers(veh_id, followers)
                    # print(f"Vehicle {veh_id}: lane_leaders={leaders}, lane_headways={headways}")
                else:
                    num_lanes = self.master_kernel.network.num_lanes(edge) if edge else max_lanes
                    self.set_lane_leaders(veh_id, [None] * num_lanes)
                    self.set_lane_headways(veh_id, [1000.0] * num_lanes)
                    self.set_lane_tailways(veh_id, [1000.0] * num_lanes)
                    self.set_lane_followers(veh_id, [None] * num_lanes)
                    print(f"Default lane_leaders for {veh_id}: {[None] * num_lanes}")
            except Exception as e:
                print(f"Error in _multi_lane_headways for {veh_id}: {e}")
                num_lanes = self.master_kernel.network.num_lanes(edge) if edge else max_lanes
                self.set_lane_leaders(veh_id, [None] * num_lanes)
                self.set_lane_headways(veh_id, [1000.0] * num_lanes)
                self.set_lane_tailways(veh_id, [1000.0] * num_lanes)
                self.set_lane_followers(veh_id, [None] * num_lanes)

        self._ids_by_edge = {edge: [] for edge in edge_list}
        for edge_id in edge_dict:
            edges = list(itertools.chain.from_iterable(edge_dict[edge_id]))
            if len(edges) > 0:
                edges, _ = zip(*edges)
                self._ids_by_edge[edge_id] = list(edges)

    def _multi_lane_headways_util(self, veh_id, edge_dict, num_edges):
        """Compute multi-lane headways, tailways, leaders, and followers for one vehicle."""
        try:
            this_pos = self.get_position(veh_id)
            this_edge = self.get_edge(veh_id)
            this_lane = self.get_lane(veh_id)
            road_len = self.master_kernel.network.length()
            num_lanes = self.master_kernel.network.num_lanes(this_edge) if this_edge else 1

            # 初始化默认值
            headway = [road_len] * num_lanes
            tailway = [road_len] * num_lanes
            leader = [None] * num_lanes
            follower = [None] * num_lanes

            # 非法车辆直接返回默认值
            if not this_edge or this_pos < 0 or this_lane < 0:
                print(f"[Warning] Invalid veh_id={veh_id}, edge={this_edge}, pos={this_pos}, lane={this_lane}")
                return headway, tailway, leader, follower

            for lane in range(num_lanes):
                lane_vehicles = edge_dict.get(this_edge, [[] for _ in range(num_lanes)])[lane]
                lane_vehicles.sort(key=lambda x: x[1])  # 按位置升序
                ids, positions = zip(*lane_vehicles) if lane_vehicles else ([], [])

                # ==== 查找 leader ====
                index = bisect_left(positions, this_pos) if positions else 0
                dx, lead_id = road_len, None

                if ids:
                    if index < len(ids):
                        candidate = ids[index]
                        pos_c = positions[index]
                        if candidate != veh_id:
                            dx = pos_c - this_pos - self.get_length(candidate)
                            lead_id = candidate
                        elif index + 1 < len(ids):
                            candidate = ids[index + 1]
                            pos_c = positions[index + 1]
                            dx = pos_c - this_pos - self.get_length(candidate)
                            lead_id = candidate

                if dx < 0 or lead_id == veh_id:
                    dx = road_len
                    lead_id = None

                headway[lane] = np.clip(dx, 0.0, road_len)
                leader[lane] = lead_id

                # ==== 查找 follower ====
                db, follow_id = road_len, None
                if ids and index > 0:
                    candidate = ids[index - 1]
                    pos_c = positions[index - 1]
                    if candidate != veh_id:
                        db = this_pos - pos_c - self.get_length(veh_id)
                        follow_id = candidate

                if db < 0 or follow_id == veh_id:
                    db = road_len
                    follow_id = None

                tailway[lane] = np.clip(db, 0.0, road_len)
                follower[lane] = follow_id

                # ==== 下一条边查找 leader ====
                if leader[lane] is None:
                    dx2, lead_id2 = self._next_edge_leaders(veh_id, edge_dict, lane, num_edges)
                    if dx2 < headway[lane]:  # 用更短的覆盖
                        headway[lane] = np.clip(dx2, 0.0, road_len)
                        leader[lane] = lead_id2

                # ==== 前一条边查找 follower ====
                if follower[lane] is None:
                    db2, follow_id2 = self._prev_edge_followers(veh_id, edge_dict, lane, num_edges)
                    if db2 < tailway[lane]:
                        tailway[lane] = np.clip(db2, 0.0, road_len)
                        follower[lane] = follow_id2

            return headway, tailway, leader, follower

        except Exception as e:
            print(f"[Error] Failed to compute headways for {veh_id}: {e}")
            road_len = self.master_kernel.network.length()
            return [road_len] * 3, [road_len] * 3, [None] * 3, [None] * 3


    def _next_edge_leaders(self, veh_id, edge_dict, lane, num_edges):
        """Search for leaders in the next edge.

        Looks to the edges/junctions in front of the vehicle's current edge
        for potential leaders. This is currently done by only looking one
        edge/junction forwards.

        Returns
        -------
        headway : float
            lane headway for the specified lane
        leader : str
            lane leader for the specified lane
        """
        pos = self.get_position(veh_id)
        edge = self.get_edge(veh_id)

        headway = 1000  # env.network.length
        leader = ""
        add_length = 0  # length increment in headway

        for _ in range(num_edges):
            # break if there are no edge/lane pairs behind the current one
            if len(self.master_kernel.network.next_edge(edge, lane)) == 0:
                break

            add_length += self.master_kernel.network.edge_length(edge)
            edge, lane = self.master_kernel.network.next_edge(edge, lane)[0]

            try:
                if len(edge_dict[edge][lane]) > 0:
                    leader = edge_dict[edge][lane][0][0]
                    headway = edge_dict[edge][lane][0][1] - pos + add_length \
                        - self.get_length(leader)
            except KeyError:
                # current edge has no vehicles, so move on
                # print(traceback.format_exc())
                continue

            # stop if a lane follower is found
            if leader != "":
                break

        return headway, leader

    def _prev_edge_followers(self, veh_id, edge_dict, lane, num_edges):
        """Search for followers in the previous edge.

        Looks to the edges/junctions behind the vehicle's current edge for
        potential followers. This is currently done by only looking one
        edge/junction backwards.

        Returns
        -------
        tailway : float
            lane tailway for the specified lane
        follower : str
            lane follower for the specified lane
        """
        pos = self.get_position(veh_id)
        edge = self.get_edge(veh_id)

        tailway = 1000  # env.network.length
        follower = ""
        add_length = 0  # length increment in headway

        for _ in range(num_edges):
            # break if there are no edge/lane pairs behind the current one
            if len(self.master_kernel.network.prev_edge(edge, lane)) == 0:
                break

            edge, lane = self.master_kernel.network.prev_edge(edge, lane)[0]
            add_length += self.master_kernel.network.edge_length(edge)

            try:
                if len(edge_dict[edge][lane]) > 0:
                    tailway = pos - edge_dict[edge][lane][-1][1] + add_length \
                              - self.get_length(veh_id)
                    follower = edge_dict[edge][lane][-1][0]
            except KeyError:
                # current edge has no vehicles, so move on
                # print(traceback.format_exc())
                continue

            # stop if a lane follower is found
            if follower != "":
                break

        return tailway, follower

    def apply_acceleration(self, veh_ids, acc, smooth=True):
        """See parent class."""
        # to handle the case of a single vehicle
        if type(veh_ids) == str:
            veh_ids = [veh_ids]
            acc = [acc]

        for i, vid in enumerate(veh_ids):
            if acc[i] is not None and vid in self.get_ids():
                self.__vehicles[vid]["accel"] = acc[i]
                this_vel = self.get_speed(vid)
                next_vel = max([this_vel + acc[i] * self.sim_step, 0])
                if smooth:
                    self.kernel_api.vehicle.slowDown(vid, next_vel, 1e-3)
                else:
                    self.kernel_api.vehicle.setSpeed(vid, next_vel)

    def apply_lane_change(self, veh_ids, direction):
        """See parent class."""
        # to hand the case of a single vehicle
        if type(veh_ids) == str:
            veh_ids = [veh_ids]
            direction = [direction]

        # if any of the directions are not -1, 0, or 1, raise a ValueError
        if any(d not in [-1, 0, 1] for d in direction):
            raise ValueError(
                "Direction values for lane changes may only be: -1, 0, or 1.")

        for i, veh_id in enumerate(veh_ids):
            # check for no lane change
            if direction[i] == 0:
                continue

            # compute the target lane, and clip it so vehicle don't try to lane
            # change out of range
            this_lane = self.get_lane(veh_id)
            this_edge = self.get_edge(veh_id)
            target_lane = min(
                max(this_lane + direction[i], 0),
                self.master_kernel.network.num_lanes(this_edge) - 1)

            # perform the requested lane action action in TraCI
            if target_lane != this_lane:
                self.kernel_api.vehicle.changeLane(
                    veh_id, int(target_lane), self.sim_step)

                if veh_id in self.get_rl_ids():
                    self.prev_last_lc[veh_id] = \
                        self.__vehicles[veh_id]["last_lc"]

    def choose_routes(self, veh_ids, route_choices):
        """See parent class."""
        # to hand the case of a single vehicle
        if type(veh_ids) == str:
            veh_ids = [veh_ids]
            route_choices = [route_choices]

        for i, veh_id in enumerate(veh_ids):
            if route_choices[i] is not None:
                self.kernel_api.vehicle.setRoute(
                    vehID=veh_id, edgeList=route_choices[i])

    def get_x_by_id(self, veh_id):
        """See parent class."""
        if isinstance(veh_id, (list, np.ndarray)):
            return [self.get_x_by_id(vehID) for vehID in veh_id]
        if self.get_edge(veh_id) == '':
            # occurs when a vehicle crashes is teleported for some other reason
            return 0.
        return self.master_kernel.network.get_x(
            self.get_edge(veh_id), self.get_position(veh_id))

    def update_vehicle_colors(self):
        """See parent class.

        The colors of all vehicles are updated as follows:
        - red: autonomous (rl) vehicles
        - white: unobserved human-driven vehicles
        - cyan: observed human-driven vehicles
        """
        for veh_id in self.get_rl_ids():
            try:
                # If vehicle is already being colored via argument to vehicles.add(), don't re-color it.
                if self._force_color_update or 'color' not in \
                        self.type_parameters[self.get_type(veh_id)]:
                    # color rl vehicles red
                    self.set_color(veh_id=veh_id, color=RED)
            except (FatalTraCIError, TraCIException) as e:
                print('Error when updating rl vehicle colors:', e)

        # color vehicles white if not observed and cyan if observed
        for veh_id in self.get_human_ids():
            try:
                color = CYAN if veh_id in self.get_observed_ids() else WHITE
                # If vehicle is already being colored via argument to vehicles.add(), don't re-color it.
                if self._force_color_update or 'color' not in \
                        self.type_parameters[self.get_type(veh_id)]:
                    self.set_color(veh_id=veh_id, color=color)
            except (FatalTraCIError, TraCIException) as e:
                print('Error when updating human vehicle colors:', e)

        for veh_id in self.get_ids():
            try:
                if 'av' in veh_id:
                    color = RED
                    # If vehicle is already being colored via argument to vehicles.add(), don't re-color it.
                    if self._force_color_update or 'color' not in \
                            self.type_parameters[self.get_type(veh_id)]:
                        self.set_color(veh_id=veh_id, color=color)
            except (FatalTraCIError, TraCIException) as e:
                print('Error when updating human vehicle colors:', e)

        # color vehicles by speed if desired
        if self._color_by_speed:
            max_speed = self.master_kernel.network.max_speed()
            speed_ranges = np.linspace(0, max_speed, STEPS)
            for veh_id in self.get_ids():
                veh_speed = self.get_speed(veh_id)
                bin_index = np.digitize(veh_speed, speed_ranges)
                # If vehicle is already being colored via argument to vehicles.add(), don't re-color it.
                if self._force_color_update or 'color' not in \
                        self.type_parameters[self.get_type(veh_id)]:
                    self.set_color(veh_id=veh_id, color=color_bins[bin_index])

        # clear the list of observed vehicles
        for veh_id in self.get_observed_ids():
            self.remove_observed(veh_id)

    def get_color(self, veh_id):
        """See parent class.

        This does not pass the last term (i.e. transparency).
        """
        r, g, b, t = self.kernel_api.vehicle.getColor(veh_id)
        return r, g, b

    def set_color(self, veh_id, color):
        """See parent class.

        The last term for sumo (transparency) is set to 255.
        """
        r, g, b = color
        self.kernel_api.vehicle.setColor(
            vehID=veh_id, color=(r, g, b, 255))

    def add(self, veh_id, type_id, edge, pos, lane, speed):
        """See parent class."""
        if veh_id in self.master_kernel.network.rts:
            # If the vehicle has its own route, use that route. This is used in
            # the case of network templates.
            route_id = 'route{}_0'.format(veh_id)
        else:
            num_routes = len(self.master_kernel.network.rts[edge])
            frac = [val[1] for val in self.master_kernel.network.rts[edge]]
            route_id = 'route{}_{}'.format(edge, np.random.choice(
                [i for i in range(num_routes)], size=1, p=frac)[0])

        self.kernel_api.vehicle.addFull(
            veh_id,
            route_id,
            typeID=str(type_id),
            departLane=str(lane),
            departPos=str(pos),
            departSpeed=str(speed))

    def get_max_speed(self, veh_id, error=-1001):
        """See parent class."""
        if isinstance(veh_id, (list, np.ndarray)):
            return [self.get_max_speed(vehID, error) for vehID in veh_id]
        return self.kernel_api.vehicle.getMaxSpeed(veh_id)

    def set_max_speed(self, veh_id, max_speed):
        """See parent class."""
        self.kernel_api.vehicle.setMaxSpeed(veh_id, max_speed)

    def get_accel(self, veh_id, noise=True, failsafe=True):
        """See parent class."""
        metric_name = 'accel'
        if noise:
            metric_name += '_with_noise'
        else:
            metric_name += '_no_noise'
        if failsafe:
            metric_name += '_with_falsafe'
        else:
            metric_name += '_no_failsafe'

        if metric_name not in self.__vehicles[veh_id]:
            self.__vehicles[veh_id][metric_name] = None
        return self.__vehicles[veh_id][metric_name]

    def update_accel(self, veh_id, accel, noise=True, failsafe=True):
        """See parent class."""
        metric_name = 'accel'
        if noise:
            metric_name += '_with_noise'
        else:
            metric_name += '_no_noise'
        if failsafe:
            metric_name += '_with_falsafe'
        else:
            metric_name += '_no_failsafe'

        self.__vehicles[veh_id][metric_name] = accel

    def get_realized_accel(self, veh_id):
        """See parent class."""
        if self.get_distance(veh_id) == 0:
            return 0
        return (self.get_speed(veh_id) - self.get_previous_speed(veh_id)) / self.sim_step

    def get_2d_position(self, veh_id, error=-1001):
        """See parent class."""
        return self.__sumo_obs.get(veh_id, {}).get(tc.VAR_POSITION, error)

    def get_distance(self, veh_id, error=-1001):
        """See parent class."""
        return self.__sumo_obs.get(veh_id, {}).get(tc.VAR_DISTANCE, error)

    def get_road_grade(self, veh_id):
        """See parent class."""
        # TODO : Brent
        return 0
