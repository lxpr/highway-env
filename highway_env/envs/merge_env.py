from typing import Dict, Text

import numpy as np
from gym.envs.registration import register

from highway_env import utils
from highway_env.envs.common.abstract import AbstractEnv
from highway_env.road.lane import LineType, StraightLane, SineLane
from highway_env.road.road import Road, RoadNetwork
from highway_env.vehicle.controller import ControlledVehicle
from highway_env.vehicle.objects import Obstacle
from highway_env.vehicle.behavior import MDPIDMVehicle


class MergeEnv(AbstractEnv):

    """
    A highway merge negotiation environment.

    The ego-vehicle is driving on a highway and approached a merge, with some vehicles incoming on the access ramp.
    It is rewarded for maintaining a high speed and avoiding collisions, but also making room for merging
    vehicles.
    """

    @classmethod
    def default_config(cls) -> dict:
        cfg = super().default_config()
        cfg.update({
            "duration": 40,  # [s]
            "collision_reward": -1,
            "right_lane_reward": 0.05, #0.1,
            "high_speed_reward": 0.4, #0.3,
            "merging_speed_reward": 0, #-0.5,
            "lane_change_reward": -0.05,
            "reward_speed_range": [0, 30],
            "time_to_collision_reward": -1,  # -1,
        })
        return cfg

    def _reward(self, action: int) -> float:
        """
        The vehicle is rewarded for driving with high speed on lanes to the right and avoiding collisions

        But an additional altruistic penalty is also suffered if any vehicle on the merging lane has a low speed.

        :param action: the action performed
        :return: the reward of the state-action transition
        """
        reward = sum(self.config.get(name, 0) * reward for name, reward in self._rewards(action).items())
        # Add the exponential time to collision reward
        finite_mdp_env = self.unwrapped.to_finite_mdp()
        reward += self.config["time_to_collision_reward"] * np.exp(-finite_mdp_env.ttc[finite_mdp_env.state])
        # return utils.lmap(reward,
        #                   [self.config["collision_reward"] + self.config["merging_speed_reward"],
        #                    self.config["high_speed_reward"] + self.config["right_lane_reward"]],
        #                   [0, 1])
        reward = utils.lmap(reward,
                          [self.config["collision_reward"] + self.config["time_to_collision_reward"] + self.config["lane_change_reward"],
                           self.config["high_speed_reward"] + self.config["right_lane_reward"]],
                          [0, 1])
        return reward

    def _rewards(self, action: int) -> Dict[Text, float]:
        return {
            "collision_reward": self.vehicle.crashed,
            "right_lane_reward": self.vehicle.lane_index[2] / 1,
            "high_speed_reward": self.vehicle.speed_index / (self.vehicle.target_speeds.size - 1),
            "lane_change_reward": action in [0, 2] #,
            # "merging_speed_reward": sum(  # Altruistic penalty
            #     (vehicle.target_speed - vehicle.speed) / vehicle.target_speed
            #     for vehicle in self.road.vehicles
            #     if vehicle.lane_index == ("b", "c", 2) and isinstance(vehicle, ControlledVehicle)
            # )
        }

    def _is_terminated(self) -> bool:
        """The episode is over when a collision occurs or when the access ramp has been passed."""
        if bool(self.vehicle.position[0] > 370):
            self.success = True
        # return self.vehicle.crashed or bool(self.vehicle.position[0] > 370)
        return self.vehicle.crashed

    def _is_truncated(self) -> bool:
        # return False
        """The episode is over if the ego vehicle crashed or the time is out."""
        return self.time >= self.config["duration"]

    def _reset(self) -> None:
        self._make_road()
        self._make_vehicles()
        self.success = False

    def _make_road(self) -> None:
        """
        Make a road composed of a straight highway and a merging lane.

        :return: the road
        """
        net = RoadNetwork()

        # Highway lanes
        ends = [150, 80, 80, 1200]  # Before, converging, merge, after
        c, s, n = LineType.CONTINUOUS_LINE, LineType.STRIPED, LineType.NONE
        y = [0, StraightLane.DEFAULT_WIDTH]
        line_type = [[c, s], [n, c]]
        line_type_merge = [[c, s], [n, s]]
        for i in range(2):
            net.add_lane("a", "b", StraightLane([0, y[i]], [sum(ends[:2]), y[i]], line_types=line_type[i]))
            net.add_lane("b", "c", StraightLane([sum(ends[:2]), y[i]], [sum(ends[:3]), y[i]], line_types=line_type_merge[i]))
            net.add_lane("c", "d", StraightLane([sum(ends[:3]), y[i]], [sum(ends), y[i]], line_types=line_type[i]))

        # Merging lane
        amplitude = 3.25
        ljk = StraightLane([0, 6.5 + 4 + 4], [ends[0], 6.5 + 4 + 4], line_types=[c, c], forbidden=True)
        lkb = SineLane(ljk.position(ends[0], -amplitude), ljk.position(sum(ends[:2]), -amplitude),
                       amplitude, 2 * np.pi / (2*ends[1]), np.pi / 2, line_types=[c, c], forbidden=True)
        lbc = StraightLane(lkb.position(ends[1], 0), lkb.position(ends[1], 0) + [ends[2], 0],
                           line_types=[n, c], forbidden=True)
        net.add_lane("j", "k", ljk)
        net.add_lane("k", "b", lkb)
        net.add_lane("b", "c", lbc)
        road = Road(network=net, np_random=self.np_random, record_history=self.config["show_trajectories"])
        road.objects.append(Obstacle(road, lbc.position(ends[2], 0)))
        self.road = road

    def _make_vehicles(self) -> None:
        """
        Populate a road with several vehicles on the highway and on the merging lane, as well as an ego-vehicle.

        :return: the ego-vehicle
        """
        road = self.road
        rng = self.np_random

        ego_vehicle = self.action_type.vehicle_class(road,
                                                     road.network.get_lane(("j", "k", 0)).position(80 + rng.uniform(high=20), 0),
                                                     speed=18 + rng.uniform(high=5))
        # ego_vehicle = MDPIDMVehicle(road,
        #                                              road.network.get_lane(("j", "k", 0)).position(
        #                                                  80 + rng.uniform(high=20), 0),
        #                                              speed=18 + rng.uniform(high=5))
        road.vehicles.append(ego_vehicle)

        other_vehicles_type = utils.class_from_path(self.config["other_vehicles_type"])
        # road.vehicles.append(other_vehicles_type(road, road.network.get_lane(("a", "b", 0)).position(90, 0), speed=29))
        # road.vehicles.append(other_vehicles_type(road, road.network.get_lane(("a", "b", 1)).position(70, 0), speed=31))
        # road.vehicles.append(other_vehicles_type(road, road.network.get_lane(("a", "b", 0)).position(5, 0), speed=31.5))
        # road.vehicles.append(other_vehicles_type(road, road.network.get_lane(("a", "b", 0)).position(90, 0), speed=20 + rng.uniform(high=12)))
        # road.vehicles.append(other_vehicles_type(road, road.network.get_lane(("a", "b", 1)).position(30, 0), speed=20 + rng.uniform(high=12)))
        # road.vehicles.append(other_vehicles_type(road, road.network.get_lane(("a", "b", 0)).position(5, 0), speed=20 + rng.uniform(high=12)))
        for _ in range(12):
            while True:
                vehicle = other_vehicles_type(road, road.network.get_lane(("a", "b", np.random.randint(low=0, high=2))).position(rng.uniform(high=180), 0),
                                                         speed=20 + rng.uniform(high=5))
                for v in self.road.vehicles:
                    if np.linalg.norm(vehicle.position - v.position) < 15 and vehicle.lane_index == v.lane_index:
                        break
                else:
                    self.road.vehicles.append(vehicle)
                    break
        #
        # merging_v = other_vehicles_type(road, road.network.get_lane(("j", "k", 0)).position(120 + rng.uniform(high=20), 0), speed=20 + rng.uniform(high=5))
        # merging_v.target_speed = 30
        # road.vehicles.append(merging_v)
        self.vehicle = ego_vehicle


register(
    id='merge-v0',
    entry_point='highway_env.envs:MergeEnv',
)
