import numpy as np
from gym.envs.registration import register

from highway_env import utils
from highway_env.envs.common.abstract import AbstractEnv
from highway_env.envs.common.action import Action
from highway_env.road.road import Road, RoadNetwork
from highway_env.utils import near_split
from highway_env.vehicle.controller import ControlledVehicle
from highway_env.vehicle.behavior import MDPIDMVehicle


class HighwayEnv(AbstractEnv):
    """
    A highway driving environment.

    The vehicle is driving on a straight highway with several lanes, and is rewarded for reaching a high speed,
    staying on the rightmost lanes and avoiding collisions.
    """

    @classmethod
    def default_config(cls) -> dict:
        config = super().default_config()
        config.update({
            "observation": {
                "type": "Kinematics",
            },
            "action": {
                "type": "DiscreteMetaAction",
            },
            "lanes_count": 4,
            "vehicles_count": 400,
            "controlled_vehicles": 1,
            "other_vehicles_type": "highway_env.vehicle.behavior.IDMVehicle",
            "aggressive_vehicle_type": "highway_env.vehicle.behavior.AggressiveCar",
            "aggressive_vehicle_type2": "highway_env.vehicle.behavior.VeryAggressiveCar",
            "perc_aggressive": 0.2, #0.4,
            "initial_lane_id": None,
            "duration": 60,  # [s]
            "ego_spacing": 2,
            "vehicles_density": 1,
            # "collision_reward": -1,    # The reward received when colliding with a vehicle.
            # "right_lane_reward": 0.1,  # The reward received when driving on the right-most lanes, linearly mapped to
            #                            # zero for other lanes.
            # "high_speed_reward": 0.4,  # The reward received when driving at full speed, linearly mapped to zero for
            #                            # lower speeds according to config["reward_speed_range"].
            # "lane_change_reward": 0,   # The reward received at each lane change action.
            # "reward_speed_range": [0, 30],
            # "offroad_terminal": False
            "collision_reward": -100,  # The reward received when colliding with a vehicle.
            "right_lane_reward": 0,  # The reward received when driving on the right-most lanes, linearly mapped to
            # zero for other lanes.
            "high_speed_reward": 0.4,  # The reward received when driving at full speed, linearly mapped to zero for
            # lower speeds according to config["reward_speed_range"].
            "lane_change_reward": -0.1,  # The reward received at each lane change action.
            "stop_reward": -0.2,  # The reward received when speed is 0
            "min_distance_reward": -1,
            # A coefficent for exponential penalty according to the distance to the closest vehicle
            "reward_speed_range": [0, 25],
            "offroad_terminal": True
        })
        return config

    def _reset(self) -> None:
        self._create_road()
        self._create_vehicles()

    def _create_road(self) -> None:
        """Create a road composed of straight adjacent lanes."""
        self.road = Road(network=RoadNetwork.straight_road_network(self.config["lanes_count"], speed_limit=30),
                         np_random=self.np_random, record_history=self.config["show_trajectories"])

    def _create_vehicles(self) -> None:
        """Create some new random vehicles of a given type, and add them on the road."""

        rng = self.np_random

        # other_vehicles_type = utils.class_from_path(self.config["other_vehicles_type"])
        other_vehicles_type = utils.class_from_path(self.config["other_vehicles_type"])
        aggro_type1 = utils.class_from_path(self.config["aggressive_vehicle_type"])
        aggro_type2 = utils.class_from_path(self.config["aggressive_vehicle_type2"])
        other_per_controlled = near_split(self.config["vehicles_count"], num_bins=self.config["controlled_vehicles"])

        self.controlled_vehicles = []

        for others in other_per_controlled:
            # controlled_vehicle = self.action_type.vehicle_class.create_random(
            #     self.road,
            #     speed=5,
            #     lane_id=self.config["initial_lane_id"],
            #     spacing=self.config["ego_spacing"]
            # )
            while True:
                controlled_vehicle = MDPIDMVehicle.create_random(
                    self.road,
                    speed=20,
                    lane_id=self.config["initial_lane_id"],
                    spacing=self.config["ego_spacing"]
                )
                # controlled_vehicle = MDPIDMVehicle(self.road, self.road.network.get_lane(("c", "a", 0)).position(0, 0), heading=np.pi, speed=20 + rng.uniform(high=10))
                if controlled_vehicle.on_road:
                    break
            controlled_vehicle.stop_time = 0
            self.controlled_vehicles.append(controlled_vehicle)
            self.road.vehicles.append(controlled_vehicle)

            num_aggressive = np.round(int(self.config["perc_aggressive"] * self.config["vehicles_count"]))
            if num_aggressive > 0:

                for _ in range(others):
                    while True:
                        random_lane_index = self.road.network.random_lane_index(rng)
                        b = np.random.randint(low=1, high=others)
                        if b < num_aggressive:
                            a = np.random.randint(low=1, high=5)
                            if a < 3:
                                vehicle = aggro_type2.make_on_lane(self.road, random_lane_index,
                                                                   longitudinal=rng.uniform(
                                                                       low=0,
                                                                       high=self.road.network.get_lane(
                                                                           random_lane_index).length
                                                                   ),
                                                                   speed=20 + rng.uniform(high=10))
                            else:
                                vehicle = aggro_type1.make_on_lane(self.road, random_lane_index,
                                                                   longitudinal=rng.uniform(
                                                                       low=0,
                                                                       high=self.road.network.get_lane(
                                                                           random_lane_index).length
                                                                   ),
                                                                   speed=20 + rng.uniform(high=10))
                        else:
                            vehicle = other_vehicles_type.make_on_lane(self.road, random_lane_index,
                                                                       longitudinal=rng.uniform(
                                                                           low=0,
                                                                           high=self.road.network.get_lane(
                                                                               random_lane_index).length
                                                                       ),
                                                                       speed=20 + rng.uniform(high=10))
                            # vehicle.randomize_behavior()
                        for v in self.road.vehicles:
                            if np.linalg.norm(vehicle.position - v.position) < 15:
                                break
                        else:
                            self.road.vehicles.append(vehicle)
                            break
            else:
                # Other vehicles
                for i in range(others):
                    while True:
                        random_lane_index = self.road.network.random_lane_index(rng)
                        vehicle = other_vehicles_type.make_on_lane(self.road, random_lane_index,
                                                                   longitudinal=rng.uniform(
                                                                       low=0,
                                                                       high=self.road.network.get_lane(
                                                                           random_lane_index).length
                                                                   ),
                                                                   speed=10 + rng.uniform(high=10))
                        # Prevent early collisions
                        for v in self.road.vehicles:
                            if np.linalg.norm(vehicle.position - v.position) < 15:
                                break
                        else:
                            self.road.vehicles.append(vehicle)
                            break
    # def _create_vehicles(self) -> None:
    #     """Create some new random vehicles of a given type, and add them on the road."""
    #     # other_vehicles_type = utils.class_from_path(self.config["other_vehicles_type"])
    #     other_vehicles_type = utils.class_from_path(self.config["other_vehicles_type"])
    #     aggro_type1 = utils.class_from_path(self.config["aggressive_vehicle_type"])
    #     aggro_type2 = utils.class_from_path(self.config["aggressive_vehicle_type2"])
    #     other_per_controlled = near_split(self.config["vehicles_count"], num_bins=self.config["controlled_vehicles"])
    #
    #     self.controlled_vehicles = []
    #
    #     for others in other_per_controlled:
    #         # controlled_vehicle = self.action_type.vehicle_class.create_random(
    #         #     self.road,
    #         #     speed=25,
    #         #     lane_id=self.config["initial_lane_id"],
    #         #     spacing=self.config["ego_spacing"]
    #         # )
    #         controlled_vehicle = MDPIDMVehicle.create_random(
    #             self.road,
    #             speed=25,
    #             lane_id=self.config["initial_lane_id"],
    #             spacing=self.config["ego_spacing"]
    #         )
    #         self.controlled_vehicles.append(controlled_vehicle)
    #         self.road.vehicles.append(controlled_vehicle)
    #
    #         num_aggressive = np.round(int(self.config["perc_aggressive"] * self.config["vehicles_count"]))
    #         if num_aggressive > 0:
    #
    #             for _ in range(others):
    #                 b= np.random.randint(low=1, high=others)
    #                 if b < num_aggressive:
    #                     a = np.random.randint(low=1, high=5)
    #                     if a < 3:
    #                         vehicle = aggro_type2.create_random(self.road, spacing=0.5 / self.config["vehicles_density"])
    #                     else:
    #                         vehicle = aggro_type1.create_random(self.road, spacing=0.5 / self.config["vehicles_density"])
    #                     self.road.vehicles.append(vehicle)
    #                 else:
    #                     vehicle = other_vehicles_type.create_random(self.road, spacing=0.5 / self.config["vehicles_density"])
    #                     # vehicle.randomize_behavior()
    #                     self.road.vehicles.append(vehicle)

    # def _create_vehicles(self) -> None:
    #     """Create some new random vehicles of a given type, and add them on the road."""
    #     # other_vehicles_type = utils.class_from_path(self.config["other_vehicles_type"])
    #     other_vehicles_type = utils.class_from_path(self.config["other_vehicles_type"])
    #     aggro_type1 = utils.class_from_path(self.config["aggressive_vehicle_type"])
    #     aggro_type2 = utils.class_from_path(self.config["aggressive_vehicle_type2"])
    #     other_per_controlled = near_split(self.config["vehicles_count"], num_bins=self.config["controlled_vehicles"])
    #
    #     self.controlled_vehicles = []
    #
    #     for others in other_per_controlled:
    #         controlled_vehicle = self.action_type.vehicle_class.create_random(
    #             self.road,
    #             speed=25,
    #             lane_id=self.config["initial_lane_id"],
    #             spacing=self.config["ego_spacing"]
    #         )
    #         self.controlled_vehicles.append(controlled_vehicle)
    #         self.road.vehicles.append(controlled_vehicle)
    #
    #         num_aggressive = np.round(int(self.config["perc_aggressive"] * self.config["vehicles_count"]))
    #         if num_aggressive > 0:
    #
    #             for _ in range(num_aggressive):
    #                 a = np.random.randint(low=1, high=5)
    #                 if a < 3:
    #                     vehicle = aggro_type2.create_random(self.road, spacing=0.5 / self.config["vehicles_density"])
    #                 else:
    #                     vehicle = aggro_type1.create_random(self.road, spacing=0.5 / self.config["vehicles_density"])
    #                 self.road.vehicles.append(vehicle)
    #         for _ in range(others-num_aggressive):
    #             vehicle = other_vehicles_type.create_random(self.road, spacing=0.5 / self.config["vehicles_density"])
    #             # vehicle.randomize_behavior()
    #             self.road.vehicles.append(vehicle)


    # def _reward(self, action: Action) -> float:
    #     """
    #     The reward is defined to foster driving at high speed, on the rightmost lanes, and to avoid collisions.
    #     :param action: the last action performed
    #     :return: the corresponding reward
    #     """
    #     neighbours = self.road.network.all_side_lanes(self.vehicle.lane_index)
    #     lane = self.vehicle.target_lane_index[2] if isinstance(self.vehicle, ControlledVehicle) \
    #         else self.vehicle.lane_index[2]
    #     scaled_speed = utils.lmap(self.vehicle.speed, self.config["reward_speed_range"], [0, 1])
    #     reward = \
    #         + self.config["collision_reward"] * self.vehicle.crashed \
    #         + self.config["right_lane_reward"] * lane / max(len(neighbours) - 1, 1) \
    #         + self.config["high_speed_reward"] * np.clip(scaled_speed, 0, 1)
    #     reward = utils.lmap(reward,
    #                       [self.config["collision_reward"],
    #                        self.config["high_speed_reward"] + self.config["right_lane_reward"]],
    #                       [0, 1])
    #     reward = 0 if not self.vehicle.on_road else reward
    #     return reward
    def _reward(self, action: Action) -> float:
        """
        The reward is defined to foster driving at high speed, on the rightmost lanes, and to avoid collisions.
        :param action: the last action performed
        :return: the corresponding reward
        """
        if self.vehicle.speed <= 0.1:
            self.vehicle.stop_time += 1
        else:
            self.vehicle.stop_time = 0
        neighbours = self.road.network.all_side_lanes(self.vehicle.lane_index)
        lane = self.vehicle.target_lane_index[2] if isinstance(self.vehicle, ControlledVehicle) \
            else self.vehicle.lane_index[2]
        scaled_speed = utils.lmap_with_limit(self.vehicle.speed, self.config["reward_speed_range"], [0, 1])
        front_vehicle, rear_vehicle = self.vehicle.road.neighbour_vehicles(self.vehicle, self.vehicle.lane_index)
        if self.vehicle.lane_index != self.vehicle.target_lane_index:
            front_vehicle, rear_vehicle = self.vehicle.road.neighbour_vehicles(self.vehicle, self.vehicle.target_lane_index)
        default_distance = 50
        if front_vehicle:
            d_front = abs(self.vehicle.lane_distance_to(front_vehicle))
        else:
            d_front = default_distance
        if rear_vehicle:
            d_rear = abs(self.vehicle.lane_distance_to(rear_vehicle))
        else:
            d_rear = default_distance
        min_distance = min(d_front, d_rear)
        # print(min_distance)
        reward = \
            + self.config["collision_reward"] * self.vehicle.crashed \
            + self.config["right_lane_reward"] * lane / max(len(neighbours) - 1, 1) \
            + self.config["high_speed_reward"] * np.clip(scaled_speed, 0, 1) \
            + self.config["stop_reward"] * self.vehicle.stop_time \
            + self.config["min_distance_reward"] * (np.exp(1 / max(0.25, (min_distance - self.vehicle.LENGTH))) - 1) * (self.vehicle.speed > 2)
        # reward = utils.lmap(reward,
        #                   [self.config["collision_reward"],
        #                    self.config["high_speed_reward"] + self.config["right_lane_reward"]],
        #                   [-1, 1])
        reward = -1 if not self.vehicle.on_road else reward
        return reward

    def _is_terminal(self) -> bool:
        """The episode is over if the ego vehicle crashed or the time is out."""
        return self.vehicle.crashed or \
            self.steps >= self.config["duration"] or \
            (self.config["offroad_terminal"] and not self.vehicle.on_road)

    def _cost(self, action: int) -> float:
        """The cost signal is the occurrence of collision."""
        return float(self.vehicle.crashed)


class HighwayEnvFast(HighwayEnv):
    """
    A variant of highway-v0 with faster execution:
        - lower simulation frequency
        - fewer vehicles in the scene (and fewer lanes, shorter episode duration)
        - only check collision of controlled vehicles with others
    """
    @classmethod
    def default_config(cls) -> dict:
        cfg = super().default_config()
        cfg.update({
            "simulation_frequency": 5,
            "lanes_count": 3,
            "vehicles_count": 20,
            "duration": 30,  # [s]
            "ego_spacing": 1.5,
        })
        return cfg

    def _create_vehicles(self) -> None:
        super()._create_vehicles()
        # Disable collision check for uncontrolled vehicles
        for vehicle in self.road.vehicles:
            if vehicle not in self.controlled_vehicles:
                vehicle.check_collisions = False


register(
    id='highway-v0',
    entry_point='highway_env.envs:HighwayEnv',
)

register(
    id='highway-fast-v0',
    entry_point='highway_env.envs:HighwayEnvFast',
)
