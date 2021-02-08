"""
v0.3 supports loading data from NuScene dataset(prediction problem)
This requires you to install NuScene PythonSDK before you can load data into PyTrafficSim
Please follow install instruction from: https://github.com/nutonomy/nuscenes-devkit#prediction-challenge

v0.3 also requires Python version >= 3.4 for the pathlib
"""
from pathlib import Path
from nuscenes import NuScenes
from nuscenes.prediction import PredictHelper
from util import *
from pyquaternion import Quaternion
from nuscenes.map_expansion import arcline_path_utils
from nuscenes.map_expansion.map_api import NuScenesMap


class NuScenesLoader:
    def __init__(self, scale=15,
                 frame_rate=30.0, window_h=1000, window_w=1000, trajectory=True,
                 collision_speculate_skip_rate=15, speed_decrease_when_crowded=1.5,
                 data_root=Path("data/sets/nuscenes/")):
        # self.map = map_cls()
        self.scale = scale
        self.window_w = window_w
        self.window_h = window_h
        self.offsets = 0, 0
        self.agents = []
        self.frame_rate = frame_rate
        self.trajectory = trajectory
        self.collision_detect = True
        self.finished_agent_id = []
        self.finished_agent = 0
        self.speed_decrease_when_crowded = speed_decrease_when_crowded  # divide 1.5 per 4 vehicles
        self.collision_speculate_skip_rate = collision_speculate_skip_rate  # the larger the faster and more likely to miss a cross collision
        self.data_root = data_root

    def load_a_scene(self, scene_num=0):
        """
        For mini_train/mini_val: Train and val splits of the mini subset used for visualization and debugging (8/2 scenes).
        {instance_token}_{sample_token}: ['bc38961ca0ac4b14ab90e547ba79fbb6_39586f9d59004284a7114a68825e8eec',
                                         'bc38961ca0ac4b14ab90e547ba79fbb6_f1e3d9d08f044c439ce86a2d6fcca57b']
        :return a list of agent variables at each frame.
        say there are 30 frames, the list has a length of 30. In each element,
        the length is the number of agents in that frame.
        In each agent, the list is 7 (x, y, yaw, width, length(the larger one), v, a)
        """

        try:
            nuscenes = NuScenes('v1.0-mini', dataroot=str(self.data_root))
        except ImportError:
            print("NuScenes not found")

        my_scene = nuscenes.scene[scene_num]
        # get all keyframes in the scene
        helper = PredictHelper(nuscenes)
        frame_list = []
        minimal_x = 999999999
        minimal_y = 999999999
        max_x = 0
        max_y = 0

        sample_tokens = nuscenes.field2token('sample', 'scene_token', my_scene["token"])
        for sample_token in sample_tokens:
            # each frame
            agent_list = []
            my_sample = nuscenes.get('sample', sample_token)
            annotation_tokens = my_sample['anns']
            for i, annotation_token in enumerate(annotation_tokens):
                # each agent
                agent_parameter_list = []
                annotation_metadata = nuscenes.get('sample_annotation', annotation_token)
                category_name = annotation_metadata['category_name']
                if category_name.split('.')[0] == 'vehicle':
                    # current version does not cover pedestrian yet
                    agent_parameter_list += annotation_metadata['translation'][:2]
                    # load quaternion rotation to yaw towards world z coordinates
                    yaw = Quaternion(annotation_metadata['rotation']).yaw_pitch_roll[0]
                    agent_parameter_list.append(normalize_angle(yaw + math.pi / 2))
                    agent_parameter_list += annotation_metadata['size'][:2]
                    # note velocity and acceleration can be None at frame 1 and 1+2
                    agent_parameter_list.append(helper.get_velocity_for_agent(
                            annotation_metadata['instance_token'],
                            annotation_metadata['sample_token']
                    ))
                    agent_parameter_list.append(helper.get_acceleration_for_agent(
                            annotation_metadata['instance_token'],
                            annotation_metadata['sample_token']
                    ))
                    agent_list.append(agent_parameter_list)
                    # handle offset
                    x, y, _ = annotation_metadata['translation']
                    if x < minimal_x:
                        minimal_x = x
                    if x > max_x:
                        max_x = x
                    if y < minimal_y:
                        minimal_y = y
                    if y > max_y:
                        max_y = y

            frame_list.append(agent_list)
        self.offsets = - abs(max_x - minimal_x) / 2 - minimal_x, - abs(max_y - minimal_y) / 2 - minimal_y
        return frame_list


'''
current version drawn trajectory of instance with the token of 'bc38961ca0ac4b14ab90e547ba79fbb6'
'''