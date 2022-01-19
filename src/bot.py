from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.messages.flat.QuickChatSelection import QuickChatSelection
from rlbot.utils.structures.ball_prediction_struct import BallPrediction, Slice
from rlbot.utils.structures.game_data_struct import GameTickPacket
from util.orientation import Orientation, relative_location

from util.boost_pad_tracker import BoostPadTracker
from util.drive import steer_toward_target
from util.sequence import Sequence, ControlStep
from util.vec import Vec3
from time import sleep, thread_time
from offense import *
from tools import *
from tests import *

class MyBot(BaseAgent):

    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        self.active_sequence: Sequence = None
        self.boost_pad_tracker = BoostPadTracker()
        self.status = 'kickoff'
        self.target_location = Vec3(0,1500,0)
        self.target_timer = 0
        self.target_slice = None
        self.target_arrival_time = 0
        self.distance_to_target = 0
        self.packet = None
        self.speed_needed = 1410
        self.ball_location = None
        self.ball_prediction = None
        self.offset_ball_location = None
        self.approach_location = None
        self.car = None
        self.location = Vec3(0,1500,0)
        self.car_velocity = None

        # edge points between top corners
        self.front_top = None
        self.rear_top = None
        self.left_top = None
        self.right_top = None
        # edge points between bottom corners
        self.front_bottom = None
        self.rear_bottom = None
        self.left_bottom = None
        self.right_bottom = None

        # top corners
        self.front_right_top_corner = None
        self.front_left_top_corner = None
        self.rear_right_top_corner = None
        self.rear_left_top_corner = None
        # bottom corners 
        self.front_right_bottom_corner = None
        self.front_left_bottom_corner = None
        self.rear_right_bottom_corner = None
        self.rear_left_bottom_corner = None
        self.car_contact_point = None
        self.ball_contact_point = None
        self.field_value_multiplier = 1 # flipping values based on team (1 or -1)
        self.time_triggered = None
        self.time_to_brake = None
        self.test_timer = None
        self.steer = 0
        self.throttle = 1
        self.dribble_distance = 1000
        self.dribble_point = None
        self.boost = False
        self.hitbox = None
        self.ball_contact_point = None
        self.dribbling = False
        self.dribble_began = 0
        self.start_location = None
        self.path = None
        self.paths = []

    def initialize_agent(self):
        # Set up information about the boost pads now that the game is active and the info is available
        self.boost_pad_tracker.initialize_boosts(self.get_field_info())

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        """
        This function will be called by the framework many times per second. This is where you can
        see the motion of the ball, etc. and return controls to drive your car.
        """
        controls = SimpleControllerState()
        # Keep our boost pad info updated with which pads are currently active
        self.boost_pad_tracker.update_boost_status(packet)

        # This is good to keep at the beginning of get_output. It will allow you to continue
        # any sequences that you may have started during a previous call to get_output.
        if self.active_sequence is not None and not self.active_sequence.done:
            controls = self.active_sequence.tick(packet)
            if controls is not None:
                return controls

        # Gather some game data
        self.car = packet.game_cars[self.index]
        self.hitbox = self.car.hitbox
        self.hitbox_offset = self.car.hitbox_offset
        self.location = Vec3(self.car.physics.location)
        self.find_car_hit_points()
        self.car_contact_point = self.front_top
        self.car_velocity = Vec3(self.car.physics.velocity)
        self.ball_location = Vec3(packet.game_ball.physics.location)
        self.ball_prediction = self.get_ball_prediction_struct()
        self.ball_contact_point = self.ball_location
        self.packet = packet
        if self.team != 1:
            self.field_value_multiplier = -1        

        # finding a shot on the opponents net
        target_location = shot_on_goal(self)

        # dribble the ball into the opponents net
        target_location = ground_dribble(self)

        # # how for from contact point of car to the dribble point on the ball
        distance_to_contact = target_location.dist(self.front_top)
         

        # # STOPPING DISTANCE STUFFS
        stopping_distance = distance_to_stop(self)
        # # start braking if we hit our minimum stopping distance
        if stopping_distance <= distance_to_contact-0:
            self.throttle = -1
            # if distance_to_contact > 100:
            #     controls.boost = True
        else:
            self.throttle = -1
            
        # limit speed to arrive at predicted target on time
        if self.car_velocity.length() > self.speed_needed:
            self.throttle = -1
        else:
            self.throttle = 1
        # steer towards target
        # target_location = Vec3(0,0,0)
        self.steer = steer_toward_target(self, target_location)
        
        # Testing path creation
        # self.path = Path(self.ball_contact_point)
        # self.path.hit_buffer(50)
        # self.path.final_approach(self.car_velocity.length(), 2000)
        # self.renderer.draw_polyline_3d(self.path.locations[::50], self.renderer.cyan())

        if abs(self.steer) == 1:
            self.throttle /= 2
            controls.boost = False

        controls.throttle = 1
        controls.steer = self.steer
        # Draw useful visuals
        # display_visuals(self, target_location)
        return controls


    

    
    # find the location of the front of the car relative to the car location
    def find_car_hit_points(self):
        # Use the orientation object to find forward 75 units from our car
        front = Orientation(self.car.physics.rotation).forward
        # front_distance = 75
        # rear_distance = -55
        front_distance = (self.hitbox.length + self.hitbox_offset.x)/2
        rear_distance = -(self.hitbox.length - self.hitbox_offset.x)/2
        right =  Orientation(self.car.physics.rotation).right
        up =  Orientation(self.car.physics.rotation).up
        width = self.hitbox.width / 2
        height = self.hitbox.height
        height_offset = 1

        self.front_top = front * front_distance + up * (height + height_offset) + self.location
        self.rear_top = front * rear_distance + up * (height + height_offset) + self.location
        self.left_top = right * -width + up * (height + height_offset) + self.location
        self.right_top = right * width + up * (height + height_offset) + self.location

        self.front_bottom = front * front_distance + up * (height_offset) + self.location
        self.rear_bottom = front * rear_distance + up * (height_offset) + self.location
        self.left_bottom = right * -width + up * (height_offset) + self.location
        self.right_bottom = right * width + up * (height_offset) + self.location

        # Corners on top of hitbox
        self.front_right_top_corner = front * front_distance + right * width + up * (height + height_offset) + self.location
        self.front_left_top_corner = front * front_distance + right * -width + up * (height + height_offset) + self.location
        self.rear_right_top_corner = front * rear_distance + right * width + up * (height + height_offset) + self.location
        self.rear_left_top_corner = front * rear_distance + right * -width + up * (height + height_offset) + self.location

        # Corners on bottom of hitbox
        self.front_right_bottom_corner = front * front_distance + right * width + up * height_offset + self.location
        self.front_left_bottom_corner = front * front_distance + right * -width + up * height_offset + self.location
        self.rear_right_bottom_corner = front * rear_distance + right * width + up * height_offset + self.location
        self.rear_left_bottom_corner = front * rear_distance + right * -width + up * height_offset + self.location


    

    def draw_ball_predictions(self, prediction):
        if prediction is not None:
            # iterate every slice
            for num in range(prediction.num_slices):
                try:
                    self.renderer.draw_line_3d(prediction.slices[num].physics.location, prediction.slices[num+1].physics.location, self.renderer.red())
                except:
                    pass

    def boost_needed(self):
        # if the bot can not make it to the target under non-boost speeds, boost
        if 1410 < self.speed_needed < 2300:
            return 1
        return 0