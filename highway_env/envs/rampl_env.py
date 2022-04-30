# # import numpy as np
# # from gym.envs.registration import register
# # 
# # from highway_env import utils
# # from highway_env.envs.common.abstract import AbstractEnv
# # from highway_env.envs.common.action import Action
# # from highway_env.road.road import Road, RoadNetwork
# # from highway_env.utils import near_split
# # from highway_env.vehicle.controller import ControlledVehicle
# # from highway_env.road.lane import LineType, StraightLane, CircularLane, SineLane
# # from highway_env.vehicle.objects import Obstacle
# # from highway_env.vehicle.behavior import MDPIDMVehicle
# # 
# # 
# # class RampLEnv(AbstractEnv):
# #     """
# #     A on/off-ramp driving environment with a longer straight lane at merging.
# # 
# #     The vehicle is driving on a on/off-ramp scenario, and is rewarded for reaching a high speed,
# #     staying on the rightmost lanes and avoiding collisions.
# #     """
# # 
# #     @classmethod
# #     def default_config(cls) -> dict:
# #         config = super().default_config()
# #         config.update({
# #             "observation": {
# #                 "type": "Kinematics",
# #             },
# #             "action": {
# #                 "type": "DiscreteMetaAction",
# #             },
# #             "lanes_count": 2,
# #             "vehicles_count": 150,
# #             "controlled_vehicles": 1,
# #             "other_vehicles_type": "highway_env.vehicle.behavior.IDMVehicle",
# #             "aggressive_vehicle_type": "highway_env.vehicle.behavior.AggressiveCar",
# #             "aggressive_vehicle_type2": "highway_env.vehicle.behavior.VeryAggressiveCar",
# #             "perc_aggressive": 0.2,  # 0.4,
# #             "initial_lane_id": None,
# #             "duration": 60,  # [s]
# #             "ego_spacing": 2,
# #             "vehicles_density": 1,
# #             "collision_reward": -1,  # The reward received when colliding with a vehicle.
# #             "right_lane_reward": 0.1,  # The reward received when driving on the right-most lanes, linearly mapped to
# #             # zero for other lanes.
# #             "high_speed_reward": 0.4,  # The reward received when driving at full speed, linearly mapped to zero for
# #             # lower speeds according to config["reward_speed_range"].
# #             "lane_change_reward": -0.1,  # The reward received at each lane change action.
# #             "stop_reward": -0.1,  # The reward received when speed is 0
# #             "reward_speed_range": [0, 20],
# #             "offroad_terminal": True
# #         })
# #         return config
# # 
# #     def _reset(self) -> None:
# #         self._create_road()
# #         self._create_vehicles()
# # 
# #     def _create_road(self) -> None:
# #         """Create a road composed of on/off-ramp lanes."""
# #         center = [0, 0]  # [m]
# #         radius = 70  # [m]
# #         alpha = 60  # [deg]
# #         straight_length = 100  # [m]
# #         lane_width = 4  # [m]
# #         straight_length_merging = 100 # [m]
# # 
# #         net = RoadNetwork()
# #         radii = [radius, radius + lane_width, radius + 2 * lane_width, radius + 3 * lane_width, radius + 4 * lane_width]
# #         n, c, s = LineType.NONE, LineType.CONTINUOUS, LineType.STRIPED
# #         line_merge = [[c, s], [n, s], [n, s], [n, s], [n, c]]
# #         line = [[c, s], [n, s], [n, c]]
# #         line_separate = [[c, s], [n, c]]
# #         for lane in [0, 1, 2]:
# #             net.add_lane("se", "ex",
# #                          CircularLane(center, radii[lane], np.deg2rad(90 - alpha), np.deg2rad(alpha),
# #                                       clockwise=False, line_types=line[lane]))
# #             net.add_lane("ex", "ee",
# #                          CircularLane(center, radii[lane], np.deg2rad(alpha), np.deg2rad(-alpha),
# #                                       clockwise=False, line_types=line[lane]))
# #             net.add_lane("ee", "nx",
# #                          CircularLane(center, radii[lane], np.deg2rad(-alpha), np.deg2rad(-90 + alpha),
# #                                       clockwise=False, line_types=line[lane]))
# #             net.add_lane("nx", "a",
# #                          CircularLane(center, radii[lane], np.deg2rad(-90 + alpha), np.deg2rad(-90),
# #                                       clockwise=False, line_types=line[lane]))
# #             net.add_lane("a1", "ne",
# #                          CircularLane([center[0] - straight_length_merging, center[1]], radii[lane], np.deg2rad(-90), np.deg2rad(-90 - alpha),
# #                                       clockwise=False, line_types=line[lane]))
# #             net.add_lane("ne", "wx",
# #                          CircularLane([center[0] - straight_length_merging, center[1]], radii[lane], np.deg2rad(-90 - alpha), np.deg2rad(-180 + alpha),
# #                                       clockwise=False, line_types=line[lane]))
# #             net.add_lane("wx", "we",
# #                          CircularLane([center[0] - straight_length_merging, center[1]], radii[lane], np.deg2rad(-180 + alpha), np.deg2rad(-180 - alpha),
# #                                       clockwise=False, line_types=line[lane]))
# #             net.add_lane("we", "sx",
# #                          CircularLane([center[0] - straight_length_merging, center[1]], radii[lane], np.deg2rad(180 - alpha), np.deg2rad(90 + alpha),
# #                                       clockwise=False, line_types=line[lane]))
# #             net.add_lane("sx", "b1",
# #                          CircularLane([center[0] - straight_length_merging, center[1]], radii[lane], np.deg2rad(90 + alpha), np.deg2rad(90),
# #                                       clockwise=False, line_types=line[lane]))
# #             net.add_lane("b", "se",
# #                          CircularLane(center, radii[lane], np.deg2rad(90), np.deg2rad(90 - alpha),
# #                                       clockwise=False, line_types=line[lane]))
# #         a_a1 = []
# #         for lane in range(0, 5):
# #             a_a1.append(StraightLane([center[0], center[1] - radii[lane]],
# #                                       [center[0] - straight_length_merging, center[1] - radii[lane]],
# #                                       line_types=line_merge[lane], forbidden=True))
# #             net.add_lane("a", "a1", a_a1[lane])
# #             net.add_lane("b1", "b",
# #                          StraightLane([center[0] - straight_length_merging, center[1] + radii[lane]],
# #                                       [center[0], center[1] + radii[lane]],
# #                                       line_types=line_merge[lane], forbidden=True))
# #         for lane in [3, 4]:
# #             # a_ne.append(CircularLane(center, radii[lane], np.deg2rad(-90), np.deg2rad(-90 - alpha),
# #             #                          clockwise=False, line_types=line_merge[lane]))
# #             # # net.add_lane("a", "ne",
# #             # #              CircularLane(center, radii[lane], np.deg2rad(-90), np.deg2rad(-90 - alpha),
# #             # #                           clockwise=False, line_types=line_merge[lane]))
# #             # net.add_lane("a", "ne", a_ne[lane - 3])
# #             # net.add_lane("sx", "b",
# #             #              CircularLane(center, radii[lane], np.deg2rad(90 + alpha), np.deg2rad(90),
# #             #                           clockwise=False, line_types=line_merge[lane]))
# #             net.add_lane("a", "c",
# #                          StraightLane([center[0], center[1] + radii[lane]],
# #                                       [center[0] + straight_length, center[1] + radii[lane]],
# #                                       line_types=line_separate[lane - 3], forbidden=True))
# #             net.add_lane("c", "m",
# #                          CircularLane([center[0] + straight_length, center[1]], radii[lane], np.deg2rad(90),
# #                                       np.deg2rad(0),
# #                                       clockwise=False, line_types=line_separate[lane - 3]))
# #             net.add_lane("m", "d",
# #                          CircularLane([center[0] + straight_length, center[1]], radii[lane], np.deg2rad(0),
# #                                       np.deg2rad(-90),
# #                                       clockwise=False, line_types=line_separate[lane - 3]))
# #             net.add_lane("d", "b",
# #                          StraightLane([center[0] + straight_length, center[1] - radii[lane]],
# #                                       [center[0], center[1] - radii[lane]],
# #                                       line_types=line_separate[lane - 3], forbidden=True))
# #         road = Road(network=net, np_random=self.np_random, record_history=self.config["show_trajectories"])
# #         for lane in [3, 4]:
# #             road.objects.append(Obstacle(road, a_a1[lane].position(a_a1[lane].length, 0)))
# #         self.road = road
# # 
# #     def _create_vehicles(self) -> None:
# #         """Create some new random vehicles of a given type, and add them on the road."""
# # 
# #         rng = self.np_random
# # 
# #         # other_vehicles_type = utils.class_from_path(self.config["other_vehicles_type"])
# #         other_vehicles_type = utils.class_from_path(self.config["other_vehicles_type"])
# #         aggro_type1 = utils.class_from_path(self.config["aggressive_vehicle_type"])
# #         aggro_type2 = utils.class_from_path(self.config["aggressive_vehicle_type2"])
# #         other_per_controlled = near_split(self.config["vehicles_count"], num_bins=self.config["controlled_vehicles"])
# # 
# #         self.controlled_vehicles = []
# # 
# #         for others in other_per_controlled:
# #             # controlled_vehicle = self.action_type.vehicle_class.create_random(
# #             #     self.road,
# #             #     speed=5,
# #             #     lane_id=self.config["initial_lane_id"],
# #             #     spacing=self.config["ego_spacing"]
# #             # )
# #             controlled_vehicle = MDPIDMVehicle.create_random(
# #                 self.road,
# #                 speed=20,
# #                 lane_id=self.config["initial_lane_id"],
# #                 spacing=self.config["ego_spacing"]
# #             )
# #             self.controlled_vehicles.append(controlled_vehicle)
# #             self.road.vehicles.append(controlled_vehicle)
# # 
# #             num_aggressive = np.round(int(self.config["perc_aggressive"] * self.config["vehicles_count"]))
# #             if num_aggressive > 0:
# # 
# #                 for _ in range(others):
# #                     random_lane_index = self.road.network.random_lane_index(rng)
# #                     b = np.random.randint(low=1, high=others)
# #                     if b < num_aggressive:
# #                         a = np.random.randint(low=1, high=5)
# #                         if a < 3:
# #                             vehicle = aggro_type2.make_on_lane(self.road, random_lane_index,
# #                                                                longitudinal=rng.uniform(
# #                                                                    low=0,
# #                                                                    high=self.road.network.get_lane(
# #                                                                        random_lane_index).length
# #                                                                ),
# #                                                                speed=10 + rng.uniform(high=10))
# #                         else:
# #                             vehicle = aggro_type1.make_on_lane(self.road, random_lane_index,
# #                                                                longitudinal=rng.uniform(
# #                                                                    low=0,
# #                                                                    high=self.road.network.get_lane(
# #                                                                        random_lane_index).length
# #                                                                ),
# #                                                                speed=10 + rng.uniform(high=10))
# #                     else:
# #                         vehicle = other_vehicles_type.make_on_lane(self.road, random_lane_index,
# #                                                                    longitudinal=rng.uniform(
# #                                                                        low=0,
# #                                                                        high=self.road.network.get_lane(
# #                                                                            random_lane_index).length
# #                                                                    ),
# #                                                                    speed=10 + rng.uniform(high=10))
# #                         # vehicle.randomize_behavior()
# #                     for v in self.road.vehicles:
# #                         if np.linalg.norm(vehicle.position - v.position) < 15:
# #                             break
# #                     else:
# #                         self.road.vehicles.append(vehicle)
# #             else:
# #                 # Other vehicles
# #                 for i in range(others):
# #                     random_lane_index = self.road.network.random_lane_index(rng)
# #                     vehicle = other_vehicles_type.make_on_lane(self.road, random_lane_index,
# #                                                                longitudinal=rng.uniform(
# #                                                                    low=0,
# #                                                                    high=self.road.network.get_lane(
# #                                                                        random_lane_index).length
# #                                                                ),
# #                                                                speed=10 + rng.uniform(high=10))
# #                     # Prevent early collisions
# #                     for v in self.road.vehicles:
# #                         if np.linalg.norm(vehicle.position - v.position) < 15:
# #                             break
# #                     else:
# #                         self.road.vehicles.append(vehicle)
# # 
# #     # def _create_vehicles(self) -> None:
# #     #     """Create some new random vehicles of a given type, and add them on the road."""
# #     #     # other_vehicles_type = utils.class_from_path(self.config["other_vehicles_type"])
# #     #     other_vehicles_type = utils.class_from_path(self.config["other_vehicles_type"])
# #     #     aggro_type1 = utils.class_from_path(self.config["aggressive_vehicle_type"])
# #     #     aggro_type2 = utils.class_from_path(self.config["aggressive_vehicle_type2"])
# #     #     other_per_controlled = near_split(self.config["vehicles_count"], num_bins=self.config["controlled_vehicles"])
# #     #
# #     #     self.controlled_vehicles = []
# #     #
# #     #     for others in other_per_controlled:
# #     #         controlled_vehicle = self.action_type.vehicle_class.create_random(
# #     #             self.road,
# #     #             speed=25,
# #     #             lane_id=self.config["initial_lane_id"],
# #     #             spacing=self.config["ego_spacing"]
# #     #         )
# #     #         self.controlled_vehicles.append(controlled_vehicle)
# #     #         self.road.vehicles.append(controlled_vehicle)
# #     #
# #     #         num_aggressive = np.round(int(self.config["perc_aggressive"] * self.config["vehicles_count"]))
# #     #         if num_aggressive > 0:
# #     #
# #     #             for _ in range(num_aggressive):
# #     #                 a = np.random.randint(low=1, high=5)
# #     #                 if a < 3:
# #     #                     vehicle = aggro_type2.create_random(self.road, spacing=0.5 / self.config["vehicles_density"])
# #     #                 else:
# #     #                     vehicle = aggro_type1.create_random(self.road, spacing=0.5 / self.config["vehicles_density"])
# #     #                 self.road.vehicles.append(vehicle)
# #     #         for _ in range(others-num_aggressive):
# #     #             vehicle = other_vehicles_type.create_random(self.road, spacing=0.5 / self.config["vehicles_density"])
# #     #             # vehicle.randomize_behavior()
# #     #             self.road.vehicles.append(vehicle)
# # 
# #     def _reward(self, action: Action) -> float:
# #         """
# #         The reward is defined to foster driving at high speed, on the rightmost lanes, and to avoid collisions.
# #         :param action: the last action performed
# #         :return: the corresponding reward
# #         """
# #         neighbours = self.road.network.all_side_lanes(self.vehicle.lane_index)
# #         lane = self.vehicle.target_lane_index[2] if isinstance(self.vehicle, ControlledVehicle) \
# #             else self.vehicle.lane_index[2]
# #         scaled_speed = utils.lmap_with_limit(self.vehicle.speed, self.config["reward_speed_range"], [0, 1])
# #         reward = \
# #             + self.config["collision_reward"] * self.vehicle.crashed \
# #             + self.config["right_lane_reward"] * lane / max(len(neighbours) - 1, 1) \
# #             + self.config["high_speed_reward"] * np.clip(scaled_speed, 0, 1) \
# #             + self.config["stop_reward"] * (self.vehicle.speed <= 0)
# #         reward = utils.lmap(reward,
# #                             [self.config["collision_reward"],
# #                              self.config["high_speed_reward"] + self.config["right_lane_reward"]],
# #                             [-1, 1])
# #         reward = -1 if not self.vehicle.on_road else reward
# #         return reward
# # 
# #     def _is_terminal(self) -> bool:
# #         """The episode is over if the ego vehicle crashed or the time is out."""
# #         return self.vehicle.crashed or \
# #                self.steps >= self.config["duration"] or \
# #                (self.config["offroad_terminal"] and not self.vehicle.on_road)
# # 
# #     def _cost(self, action: int) -> float:
# #         """The cost signal is the occurrence of collision."""
# #         return float(self.vehicle.crashed)
# # 
# # 
# # register(
# #     id='ramp-v1',
# #     entry_point='highway_env.envs:RampLEnv',
# # )
# 
# import numpy as np
# from gym.envs.registration import register
# 
# from highway_env import utils
# from highway_env.envs.common.abstract import AbstractEnv
# from highway_env.envs.common.action import Action
# from highway_env.road.road import Road, RoadNetwork
# from highway_env.utils import near_split
# from highway_env.vehicle.controller import ControlledVehicle
# from highway_env.road.lane import LineType, StraightLane, CircularLane, SineLane
# from highway_env.vehicle.objects import Obstacle
# from highway_env.vehicle.behavior import MDPIDMVehicle
# 
# 
# class RampEnv(AbstractEnv):
#     """
#     A on/off-ramp driving environment.
# 
#     The vehicle is driving on a on/off-ramp scenario, and is rewarded for reaching a high speed,
#     staying on the rightmost lanes and avoiding collisions.
#     """
# 
#     @classmethod
#     def default_config(cls) -> dict:
#         config = super().default_config()
#         config.update({
#             "observation": {
#                 "type": "Kinematics",
#             },
#             "action": {
#                 "type": "DiscreteMetaAction",
#             },
#             "lanes_count": 2,
#             "vehicles_count": 150,
#             "controlled_vehicles": 1,
#             "other_vehicles_type": "highway_env.vehicle.behavior.IDMVehicle",
#             "aggressive_vehicle_type": "highway_env.vehicle.behavior.AggressiveCar",
#             "aggressive_vehicle_type2": "highway_env.vehicle.behavior.VeryAggressiveCar",
#             "perc_aggressive": 0.2,  # 0.4,
#             "initial_lane_id": None,
#             "duration": 60,  # [s]
#             "ego_spacing": 2,
#             "vehicles_density": 1,
#             "collision_reward": -100,  # The reward received when colliding with a vehicle.
#             "right_lane_reward": 0,  # The reward received when driving on the right-most lanes, linearly mapped to
#             # zero for other lanes.
#             "high_speed_reward": 0.4,  # The reward received when driving at full speed, linearly mapped to zero for
#             # lower speeds according to config["reward_speed_range"].
#             "lane_change_reward": -0.1,  # The reward received at each lane change action.
#             "stop_reward": -0.2,  # The reward received when speed is 0
#             "min_distance_reward": -1,
#             # A coefficent for exponential penalty according to the distance to the closest vehicle
#             "reward_speed_range": [0, 25],
#             "offroad_terminal": True
#         })
#         return config
# 
#     def _reset(self) -> None:
#         self._create_road()
#         self._create_vehicles()
# 
#     def _create_road(self) -> None:
#         """Create a road composed of on/off-ramp lanes."""
#         center = [0, 0]  # [m]
#         radius = 300  # [m]
#         alpha = 30  # [deg]
#         straight_length = 100  # [m]
#         lane_width = 4  # [m]
# 
#         net = RoadNetwork()
#         radii = [radius, radius + lane_width, radius + 2 * lane_width, radius + 3 * lane_width, radius + 4 * lane_width]
#         n, c, s = LineType.NONE, LineType.CONTINUOUS, LineType.STRIPED
#         line_merge = [[c, s], [n, s], [n, s], [n, s], [n, c]]
#         line = [[c, s], [n, s], [n, c]]
#         line_separate = [[c, s], [n, c]]
#         for lane in [0, 1, 2]:
#             net.add_lane("nx", "a",
#                          CircularLane(center, radii[lane], np.deg2rad(0), np.deg2rad(-90),
#                                       clockwise=False, line_types=line[lane], forbidden=True))
#             net.add_lane("a", "ne",
#                          CircularLane(center, radii[lane], np.deg2rad(-90), np.deg2rad(-90 - alpha),
#                                       clockwise=False, line_types=line_merge[lane], forbidden=True))
#             net.add_lane("ne", "sx",
#                          CircularLane(center, radii[lane], np.deg2rad(-90), np.deg2rad(-270 + alpha),
#                                       clockwise=False, line_types=line[lane]))
#             net.add_lane("sx", "b",
#                          CircularLane(center, radii[lane], np.deg2rad(90 + alpha), np.deg2rad(90),
#                                       clockwise=False, line_types=line_merge[lane]))
#             net.add_lane("b", "nx",
#                          CircularLane(center, radii[lane], np.deg2rad(90), np.deg2rad(0),
#                                       clockwise=False, line_types=line[lane]))
#         ne_ne1 = CircularLane(center, radii[3], np.deg2rad(-90 - alpha * 3 / 4), np.deg2rad(-90 - alpha),
#                               clockwise=False, line_types=line_merge[4])
#         net.add_lane("ne", "ne1", ne_ne1)
#         net.add_lane("a", "ne", CircularLane(center, radii[3], np.deg2rad(-90), np.deg2rad(-90 - alpha * 3 / 4),
#                                              clockwise=False, line_types=line_merge[3]))
#         a_ne = CircularLane(center, radii[4], np.deg2rad(-90), np.deg2rad(-90 - alpha * 3 / 4),
#                             clockwise=False, line_types=line_merge[4])
#         net.add_lane("a", "ne", a_ne)
#         for lane in range(3, 5):
#             # a_ne.append(CircularLane(center, radii[lane], np.deg2rad(-90), np.deg2rad(-90 - alpha * (7 - lane) / 4),
#             #                           clockwise=False, line_types=line_merge[lane]))
#             # # net.add_lane("a", "ne",
#             # #              CircularLane(center, radii[lane], np.deg2rad(-90), np.deg2rad(-90 - alpha),
#             # #                           clockwise=False, line_types=line_merge[lane]))
#             # net.add_lane("a", "ne", a_ne[lane - 3])
#             net.add_lane("sx", "b",
#                          CircularLane(center, radii[lane], np.deg2rad(90 + alpha), np.deg2rad(90),
#                                       clockwise=False, line_types=line_merge[lane]))
#             net.add_lane("c", "a",
#                          StraightLane([center[0] + straight_length, center[1] - radii[lane]],
#                                       [center[0], center[1] - radii[lane]],
#                                       line_types=line_separate[lane - 3], forbidden=True))
#             net.add_lane("m", "c",
#                          CircularLane([center[0] + straight_length, center[1]], radii[lane], np.deg2rad(0),
#                                       np.deg2rad(-90),
#                                       clockwise=False, line_types=line_separate[lane - 3]))
#             net.add_lane("d", "m",
#                          CircularLane([center[0] + straight_length, center[1]], radii[lane], np.deg2rad(90),
#                                       np.deg2rad(0),
#                                       clockwise=False, line_types=line_separate[lane - 3]))
#             net.add_lane("b", "d",
#                          StraightLane([center[0], center[1] + radii[lane]],
#                                       [center[0] + straight_length, center[1] + radii[lane]],
#                                       line_types=line_separate[lane - 3], forbidden=False))
#         road = Road(network=net, np_random=self.np_random, record_history=self.config["show_trajectories"])
#         # for lane in [3, 4]:
#         #     road.objects.append(Obstacle(road, a_ne[lane - 3].position(a_ne[lane - 3].length, 0)))
#         road.objects.append(Obstacle(road, a_ne.position(a_ne.length, 0)))
#         road.objects.append(Obstacle(road, ne_ne1.position(ne_ne1.length, 0)))
#         self.road = road
# 
#     def _create_vehicles(self) -> None:
#         """Create some new random vehicles of a given type, and add them on the road."""
# 
#         rng = self.np_random
# 
#         # other_vehicles_type = utils.class_from_path(self.config["other_vehicles_type"])
#         other_vehicles_type = utils.class_from_path(self.config["other_vehicles_type"])
#         aggro_type1 = utils.class_from_path(self.config["aggressive_vehicle_type"])
#         aggro_type2 = utils.class_from_path(self.config["aggressive_vehicle_type2"])
#         other_per_controlled = near_split(self.config["vehicles_count"], num_bins=self.config["controlled_vehicles"])
# 
#         self.controlled_vehicles = []
# 
#         for others in other_per_controlled:
#             # controlled_vehicle = self.action_type.vehicle_class.create_random(
#             #     self.road,
#             #     speed=5,
#             #     lane_id=self.config["initial_lane_id"],
#             #     spacing=self.config["ego_spacing"]
#             # )
#             while True:
#                 # controlled_vehicle = MDPIDMVehicle.create_random(
#                 #     self.road,
#                 #     speed=20,
#                 #     lane_id=self.config["initial_lane_id"],
#                 #     spacing=self.config["ego_spacing"]
#                 # )
#                 controlled_vehicle = MDPIDMVehicle(self.road, self.road.network.get_lane(("c", "a", 0)).position(0, 0),
#                                                    heading=np.pi, speed=20 + rng.uniform(high=10))
#                 if controlled_vehicle.on_road:
#                     break
#             controlled_vehicle.stop_time = 0
#             self.controlled_vehicles.append(controlled_vehicle)
#             self.road.vehicles.append(controlled_vehicle)
# 
#             num_aggressive = np.round(int(self.config["perc_aggressive"] * self.config["vehicles_count"]))
#             if num_aggressive > 0:
# 
#                 for _ in range(others):
#                     random_lane_index = self.road.network.random_lane_index(rng)
#                     b = np.random.randint(low=1, high=others)
#                     if b < num_aggressive:
#                         a = np.random.randint(low=1, high=5)
#                         if a < 3:
#                             vehicle = aggro_type2.make_on_lane(self.road, random_lane_index,
#                                                                longitudinal=rng.uniform(
#                                                                    low=0,
#                                                                    high=self.road.network.get_lane(
#                                                                        random_lane_index).length
#                                                                ),
#                                                                speed=20 + rng.uniform(high=10))
#                         else:
#                             vehicle = aggro_type1.make_on_lane(self.road, random_lane_index,
#                                                                longitudinal=rng.uniform(
#                                                                    low=0,
#                                                                    high=self.road.network.get_lane(
#                                                                        random_lane_index).length
#                                                                ),
#                                                                speed=20 + rng.uniform(high=10))
#                     else:
#                         vehicle = other_vehicles_type.make_on_lane(self.road, random_lane_index,
#                                                                    longitudinal=rng.uniform(
#                                                                        low=0,
#                                                                        high=self.road.network.get_lane(
#                                                                            random_lane_index).length
#                                                                    ),
#                                                                    speed=20 + rng.uniform(high=10))
#                         # vehicle.randomize_behavior()
#                     for v in self.road.vehicles:
#                         if np.linalg.norm(vehicle.position - v.position) < 15:
#                             break
#                     else:
#                         self.road.vehicles.append(vehicle)
#             else:
#                 # Other vehicles
#                 for i in range(others):
#                     random_lane_index = self.road.network.random_lane_index(rng)
#                     vehicle = other_vehicles_type.make_on_lane(self.road, random_lane_index,
#                                                                longitudinal=rng.uniform(
#                                                                    low=0,
#                                                                    high=self.road.network.get_lane(
#                                                                        random_lane_index).length
#                                                                ),
#                                                                speed=10 + rng.uniform(high=10))
#                     # Prevent early collisions
#                     for v in self.road.vehicles:
#                         if np.linalg.norm(vehicle.position - v.position) < 15:
#                             break
#                     else:
#                         self.road.vehicles.append(vehicle)
# 
#     # def _create_vehicles(self) -> None:
#     #     """Create some new random vehicles of a given type, and add them on the road."""
#     #     # other_vehicles_type = utils.class_from_path(self.config["other_vehicles_type"])
#     #     other_vehicles_type = utils.class_from_path(self.config["other_vehicles_type"])
#     #     aggro_type1 = utils.class_from_path(self.config["aggressive_vehicle_type"])
#     #     aggro_type2 = utils.class_from_path(self.config["aggressive_vehicle_type2"])
#     #     other_per_controlled = near_split(self.config["vehicles_count"], num_bins=self.config["controlled_vehicles"])
#     #
#     #     self.controlled_vehicles = []
#     #
#     #     for others in other_per_controlled:
#     #         controlled_vehicle = self.action_type.vehicle_class.create_random(
#     #             self.road,
#     #             speed=25,
#     #             lane_id=self.config["initial_lane_id"],
#     #             spacing=self.config["ego_spacing"]
#     #         )
#     #         self.controlled_vehicles.append(controlled_vehicle)
#     #         self.road.vehicles.append(controlled_vehicle)
#     #
#     #         num_aggressive = np.round(int(self.config["perc_aggressive"] * self.config["vehicles_count"]))
#     #         if num_aggressive > 0:
#     #
#     #             for _ in range(num_aggressive):
#     #                 a = np.random.randint(low=1, high=5)
#     #                 if a < 3:
#     #                     vehicle = aggro_type2.create_random(self.road, spacing=0.5 / self.config["vehicles_density"])
#     #                 else:
#     #                     vehicle = aggro_type1.create_random(self.road, spacing=0.5 / self.config["vehicles_density"])
#     #                 self.road.vehicles.append(vehicle)
#     #         for _ in range(others-num_aggressive):
#     #             vehicle = other_vehicles_type.create_random(self.road, spacing=0.5 / self.config["vehicles_density"])
#     #             # vehicle.randomize_behavior()
#     #             self.road.vehicles.append(vehicle)
# 
#     def _reward(self, action: Action) -> float:
#         """
#         The reward is defined to foster driving at high speed, on the rightmost lanes, and to avoid collisions.
#         :param action: the last action performed
#         :return: the corresponding reward
#         """
#         if self.vehicle.speed <= 0.1:
#             self.vehicle.stop_time += 1
#         else:
#             self.vehicle.stop_time = 0
#         neighbours = self.road.network.all_side_lanes(self.vehicle.lane_index)
#         lane = self.vehicle.target_lane_index[2] if isinstance(self.vehicle, ControlledVehicle) \
#             else self.vehicle.lane_index[2]
#         scaled_speed = utils.lmap_with_limit(self.vehicle.speed, self.config["reward_speed_range"], [0, 1])
#         front_vehicle, rear_vehicle = self.vehicle.road.neighbour_vehicles(self.vehicle, self.vehicle.lane_index)
#         if self.vehicle.lane_index != self.vehicle.target_lane_index:
#             front_vehicle, rear_vehicle = self.vehicle.road.neighbour_vehicles(self.vehicle,
#                                                                                self.vehicle.target_lane_index)
#         default_distance = 50
#         if front_vehicle:
#             d_front = abs(self.vehicle.lane_distance_to(front_vehicle))
#         else:
#             d_front = default_distance
#         if rear_vehicle:
#             d_rear = abs(self.vehicle.lane_distance_to(rear_vehicle))
#         else:
#             d_rear = default_distance
#         min_distance = min(d_front, d_rear)
#         # print(min_distance)
#         reward = \
#             + self.config["collision_reward"] * self.vehicle.crashed \
#             + self.config["right_lane_reward"] * lane / max(len(neighbours) - 1, 1) \
#             + self.config["high_speed_reward"] * np.clip(scaled_speed, 0, 1) \
#             + self.config["stop_reward"] * self.vehicle.stop_time \
#             + self.config["min_distance_reward"] * (np.exp(1 / max(0.25, (min_distance - self.vehicle.LENGTH))) - 1) * (
#                         self.vehicle.speed > 5)
#         # reward = utils.lmap(reward,
#         #                   [self.config["collision_reward"],
#         #                    self.config["high_speed_reward"] + self.config["right_lane_reward"]],
#         #                   [-1, 1])
#         reward = -1 if not self.vehicle.on_road else reward
#         return reward
# 
#     def _is_terminal(self) -> bool:
#         """The episode is over if the ego vehicle crashed or the time is out."""
#         return self.vehicle.crashed or \
#                self.steps >= self.config["duration"] or \
#                (self.config["offroad_terminal"] and not self.vehicle.on_road)
# 
#     def _cost(self, action: int) -> float:
#         """The cost signal is the occurrence of collision."""
#         return float(self.vehicle.crashed)
# 
# 
# register(
#     id='ramp-v1',
#     entry_point='highway_env.envs:RampEnv',
# )