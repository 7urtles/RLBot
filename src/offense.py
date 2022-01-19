from tools import distance_to_stop
from util.orientation import Orientation, relative_location
from util.ball_prediction_analysis import find_matching_slice
from util.drive import steer_toward_target

from util.vec import Vec3
import copy, math
# offensive decision making

# Work on making the self dribble the ball around the field
def ground_dribble(self):
    """Dribbling strategy and maneuvers"""
    # if we are close enough to dribble
    if self.location.dist(self.ball_location) > 250:
        self.dribbling = False
    # if we have dribbling posession
    if self.dribbling:
        # try and push ball into goal
        dribble_to_goal(self)
    # if we dont have posession
    else:
        # head to the ball to gain posession
        self.dribble_point = self.ball_location
        go_to_ball(self)
    return closest_point_on_ball(self, self.dribble_point)



def dribble_to_goal(self):
    """Controls car to steer ball towards opponents net"""
    # if car gets too far from ball
    if self.car_contact_point.dist(self.dribble_point) > 200:
        # The dribble has failed
        self.dribbling = False
        return False
    # retrieve angles needed to know where ball is headed in relation to a target
    max_angle, current_angle = dribble_target_angle(self)
    # contact ball on point opposite of opponents net
    self.dribble_point = shooting_location(self)
    # if ball is not headed towards target
    if abs(current_angle) > max_angle:
        # if missing right
        if current_angle > 0:
            # use appropriate corner to steer it left
            self.car_contact_point = self.front_left_top_corner
        # if missing left
        elif current_angle < 0:
            # use appropriate corner to steer it right
            self.car_contact_point = self.front_right_top_corner
        # if for some reason missing target but the angle is zero
        else:
            self.car_contact_point = self.front_top
        

    # try and stay near the intended dribble contact point
    if relative_location(self.car_contact_point, Orientation(self.car.physics.rotation), self.dribble_point).x > 0:
        self.throttle = 1
    else:
        self.throttle = -1


def go_to_ball(self):
    """Drive towards the ball without hitting it. Returns true if ball posession is gained"""
    if self.front_top.dist(closest_point_on_ball(self, self.front_top)) > 20:
        forward =  Orientation(self.car.physics.rotation).forward
        up =  Orientation(self.car.physics.rotation).up
        # closest point of the ball to the car
        self.dribble_point = self.ball_location.flat() + forward * -92.75 + up * self.front_top.z
        # how for from contact point of car to the dribble point on the ball
        distance_to_contact = self.dribble_point.dist(self.front_top)
        # start braking if we hit our minimum stopping distance
        stopping_distance = distance_to_stop(self)
        if stopping_distance <= distance_to_contact:
            self.throttle = 1
        else:
            self.throttle = -1
    # if is gets close enough to the ball to have posession
    if self.front_top.dist(closest_point_on_ball(self, self.front_top)) < 200:
        # and we have not started the posession timer
        if self.dribble_began <= 0:
            # store a copy of the current time as when we began having posession
            self.dribble_began = copy.deepcopy(self.packet.game_info.seconds_elapsed)
            return False
        # otherwise if the we have had posession long enough to begin a stable dribble
        elif self.packet.game_info.seconds_elapsed - self.dribble_began > .25:
            self.dribbling = True 
            return True 
    # if too far away to dribble
    else:
        # resent the posession timer
        self.dribble_began = 0
        return False
    

# find angle of ball to a target
def dribble_target_angle(self):
    """Return the angle of the ball to a target, and the maximum angle that will result in a goal"""
    # adjust the angle of below locations to be relative to the ball
    target_location = (self.ball_location.flat() - Vec3(0,-5120, 53.50).flat()).normalized()
    left_post = (self.ball_location.flat() - Vec3(0,-5120, 53.50).flat() - Vec3(400,0,0)).normalized()
    right_post = (self.ball_location.flat() - Vec3(0,-5120, 53.50).flat() + Vec3(400,0,0)).normalized()
    # find the ball 1/4 second in the future and use this to find the direction the ball is headed
    prediction_location = (self.ball_location - Vec3(self.ball_prediction.slices[60].physics.location).flat()).normalized()
    # the largest angle we can use to still score with
    max_angle = right_post.ang_to(left_post)
    # current shot angle
    current_angle = prediction_location.ang_to(target_location)
    # if we the ball is headed left of the target make the angle negative
    if (prediction_location * target_location.y).x > target_location.x:
        current_angle *= -1
    # convert from radians to degrees
    return max_angle * 57.2958, current_angle * 57.2958
    

def shooting_location(self):
    """Returns point on the ball opposite of the opponents goal"""
    ballHitPos = self.ball_location + Vec3(self.ball_location - Vec3(0,-5120, 53.50)).normalized() * 92.75
    # self.renderer.draw_rect_3d(ballHitPos, 10, 10, True, self.renderer.black(), centered=True)
    return ballHitPos

# find spot on ball closest to a given location
def closest_point_on_ball(self, target):
    """Returns closest point on the ball to a location"""
    ballHitPos = self.ball_location - Vec3(self.ball_location - target).normalized() * 92.75
    self.ball_contact_point = ballHitPos
    return ballHitPos
    

# if the ball goes within a zone in front of the opponents net, try to take a shot on their goal
def shot_on_goal(self):
    """Attempts to either shoot the ball on the goal, or wait midfield until a 
        suitable shot is found"""
    target_location = Vec3(0,1500*self.field_value_multiplier,0)        
    # check if target is still valid, if not a new one is found
    if update_target(self) == True:
        if self.target_location is not None and self.target_slice is not None:
            # current target is still valid, update it's timer
            update_timer(self)
            # go for the ball if there is a hit available
            if possible_shot_zone(self) == True:
                # print('ball in shot zone')
                try:
                    point_of_contact(self)
                    # check our angle to the ideal point of contact on the ball
                    current_shot_angle = check_hit_angle(self)
                    # if the current shot angle shootable
                    if current_shot_angle < 90:
                        # print('shot within angle')
                        # and the ball is in front of the car
                        target_relative_to_car = relative_location(self.car_contact_point, Orientation(self.car.physics.rotation), self.ball_contact_point)
                        if target_relative_to_car.x > 0:
                            # finalize target location
                            target_location = self.ball_contact_point
                            update_car_contact_point(self)
                        else:
                            # print('not facing target')
                            pass
                    else:
                        # print('angle to obtuse to shoot')
                        pass
                except:
                    # print('whoops')
                    pass
    else:
        # if time for ball to arrive at expected location passes
        if self.target_timer <= 0:
            # the target location is now invalid so reset it accordingly
            self.target_location = None
            # set the target location to upfield from opponets net to wait for a viable shot
            target_location = Vec3(0,1500*self.field_value_multiplier,0)
    # return new movement location
    return target_location

# find where to hit the ball that will result in it going towards opponents goal
def point_of_contact(self, left_target_location = Vec3(200, -5213, 0), right_target_location = Vec3(-200, -5213, 0)):
    """Returns a location that will result in the car slowly circling the ball towards a contact point 
        that will result in the ball heading towards the opponents net"""
    target_location = self.target_location
    car_to_ball = target_location - self.car_contact_point
    car_to_ball_direction = car_to_ball.normalized()
    ball_to_left_target_direction = (left_target_location * self.field_value_multiplier - target_location).normalized()
    ball_to_right_target_direction = (right_target_location * self.field_value_multiplier - target_location).normalized()
    
    direction_of_approach = clamp2D(car_to_ball_direction, ball_to_left_target_direction, ball_to_right_target_direction)
    offset_ball_location = target_location - (direction_of_approach * 92.75)
    self.offset_ball_location = offset_ball_location
    self.approach_location = target_location - (direction_of_approach * 500)

    side_of_approach_direction = sign(direction_of_approach.cross(Vec3(0, 0, 1)).dot(target_location - self.car_contact_point))
    car_to_ball_perpendicular = car_to_ball.cross(Vec3(0, 0, side_of_approach_direction)).normalized()
    adjustment = abs(car_to_ball.flat().ang_to(direction_of_approach.flat())) * 2560
    final_target = offset_ball_location + (car_to_ball_perpendicular * adjustment)
    self.ball_contact_point = Vec3(final_target)

def sign(number):
    """Limits a number to 1 or -1 and returns the result"""
    if number == 0:
        return 0
    elif number < 0:
        return -1
    elif number > 0:
        return 1
        
def clamp2D(direction, start, end):
    is_right = direction.dot(end.cross(Vec3(0, 0, -1))) < 0
    is_left = direction.dot(start.cross(Vec3(0, 0, -1))) > 0
    if end.dot(start.cross(Vec3(0, 0, -1))) > 0 and ((is_right and is_left) or (is_right or is_left)):
        return direction
    if start.dot(direction) < end.dot(direction):
        return end

    return start

def update_timer(self):
    """Updates the time until the ball will arrive at a predicted location"""
    # update timer
    self.distance_to_target = self.location.dist(self.target_location)
    self.target_timer = self.target_slice.game_seconds - self.packet.game_info.seconds_elapsed
    speed_needed_to_target(self, self.target_slice)

def update_target(self):
    """Checks if a future ball location is reachable before the ball arrives"""
    # returns True if target is still valid, or False if a new one is needed
    if self.target_timer <= 0 or self.target_slice is None: # or self.speed_needed > 1410
        self.target_slice = copy.deepcopy(find_nearest_hit(self))
        if self.target_slice:
            self.target_location = Vec3(self.target_slice.physics.location)
            # print('Target search failed')
            return False # new target found
    # print('No new target needed')
    return True

# pick the most appropiate location on car to hit the ball with
def update_car_contact_point(self, target):
    """Changes what point on the car will be used to hit the ball, whichever is closest to the ball"""
    contact_options = [self.front_left_top_corner, self.front_right_top_corner]
    for option in contact_options:
        if option.dist(target) < self.car_contact_point.dist(target):
            self.car_contact_point = option

def find_nearest_hit(self):
    """Looks a location on the balls prediction path that is reachable"""
    found_slice = find_matching_slice(self.ball_prediction, 0, lambda s: speed_needed_to_target(self, s) <= 1410 and s.physics.location.z <= 95, search_increment=20)
    return found_slice


def speed_needed_to_target(self, prediction_slice):
    """Finds the average speed needed to arrive at a target before a certain time"""
    #current game time
    current_time = self.packet.game_info.seconds_elapsed
    # what time the ball will arrive at the prediction
    arrival_time = prediction_slice.game_seconds
    # subract the game time to find how long until ball arrives
    time_until_arrival = arrival_time - current_time
    # stopping zero div errors
    if time_until_arrival == 0:
        return 3000
    # get distance to target
    distance_to_target = self.location.dist(Vec3(prediction_slice.physics.location))
    # time until arrival / distance to target gives average speed per tick to get to ball arrival location
    speed_needed = distance_to_target / time_until_arrival
    self.speed_needed = speed_needed
    self.target_timer = time_until_arrival
    # Return the average speed needed to arrive at the target on time
    return speed_needed

# define square on the opponents side of the field to take possible shots from
def possible_shot_zone(self):
    """Defines a shootable area on the opponents half, and returns True if the ball is within it"""
    x1 = 3000
    x2 = -3000
    if self.field_value_multiplier == 1: # if on orange
        y1 = -500
        y2 = -4500
    else: # if on blue
        y1 = 4500
        y2 = 500
    # draw the shot zone
    locations = [Vec3(x1,y1,20), Vec3(x1,y2,20), Vec3(x2,y2,20), Vec3(x2, y1,20), Vec3(x1,y1,20)]
    self.renderer.draw_polyline_3d(locations,  self.renderer.lime())
    if (x1 > self.target_location.x > x2) and (y1 > self.target_location.y > y2):
        return True
    return False

# check the angle between car to future ball location, and car to contact point of future ball location
def check_hit_angle(self):
    """Returns how well the car is facing the ball in degrees"""
    future_ball_location = self.target_location.flat()
    ball_contact_point = self.offset_ball_location.flat()-future_ball_location
    car_location = self.car_contact_point.flat()-future_ball_location
    hit_angle = ball_contact_point.ang_to(car_location) * 57.2958
    return hit_angle