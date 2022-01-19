import imp
from time import sleep
from scipy.sparse.extract import find
from util.vec import Vec3
from scipy.interpolate import interp1d, UnivariateSpline
import math, copy
import numpy as np
import pickle
from time import sleep
from util.orientation import Orientation, relative_location

# A line object containing points, curvatures, time it will take to travel the path, and other 
#   attributes useful in smartly choosing a driving path
class Path():
    def __init__(self, start_location=Vec3(1,2,20)) -> None:
        self.start_point = start_location.flat()
        self.last_curvature = 0
        self.unmodified_route = [self.start_point]
        self.buffer_start_point = None
        self.buffer_distance = None
        self.approach_distance = None
        self.locations = []
    
    # define cars final approach path to hit the ball
    def hit_buffer(self, distance=50):
        self.buffer_distance = distance   
        # set some class variables for extending the path later
        self.locations = [self.start_point + Vec3(0,0,50)]
        count = 0
        while count < distance:
            # Chain vectors outward from the intended contact location away from the goal
            self.locations.append(Vec3(self.start_point - Vec3(0,-5120, 0)).normalized() + self.locations[-1])
            count += 1
        # self.last_curvature = find_curvature(self.locations[:-3])[0]
        self.buffer_start_point = self.locations[-1]
    
    def final_approach(self, velocity=600, distance=250):
        self.extend_path(velocity, distance)

    def extend_path(self, velocity, distance):
        """Extends the existing path by a curved line segment. Receives a bot class object, velocity as a float,
        and desired arc length as an integer. Returns arc segment as a list of Vec3"""
        self.approach_distance = distance
        route = [self.locations[-1]]
        current_rotation = self.last_curvature
        # get a curvature from desired speed
        # get angle of curvature
        # create turned unit vector from angle
        # continue to chain turned unit vectors to form a path
        i=0
        while i < distance:
            # curvature from velocity
            turn_curvature = find_max_turn_curvature(velocity)
            # angle in radians to turn the next vector by, and adjusted left or right using the bots steering value
            current_rotation += turn_curvature / 2
            # Get new unit vector by applying the needed rotation to the previous vector. 
            rotated_unit_vector = route[i].normalized().rotate2D(current_rotation)
            # add new unit vector to previous one, adding it to the path
            new_point = rotated_unit_vector + route[i]
            # a seperate final list. Redundant but prevents raising the .rotate2D angle twice every loop
            route.append(new_point)
            new_point = new_point.rotate2D(Vec3(0,0,0).rel_ang_to(self.buffer_start_point, self.start_point),self.buffer_start_point)
            self.locations.append(new_point.flat() + Vec3(0,0,50))
            i+=1
        self.last_curvature = current_rotation
    
# find maximum possible speed given a curvature
def curvature_max_speed(curve):
    """takes in a curvature and return the maximium possible speed"""
    velocity_range = 2301
    max_turn_speed = None
    for velocity in range(1, velocity_range):
        # Cars turn radius given a velocity
        turn_radius_from_velocity = turn_radius(velocity)
        # turn radius is 1/curvature
        # since turn radius is provided, using turn_radius*line_curvature should == 1
        result = turn_radius_from_velocity * curve
        # if it is over or under 1, the car will over or understeer the turn
        if result < 1:
            max_turn_speed = velocity
        else:
            pass
    return max_turn_speed

# find curvature of a point on a curved line segment.
def find_curvature(line_locations):
    """Takes a list of vectors consisting of a lines points. 
    Returns curvatur values along the line as a list of floats"""
    # need two points to assume curvature so save first point and start calculating on the second
    # define point of reference for tangent vectors
    origin = Vec3(line_locations[0])
    # first tangent vector
    current_point = (Vec3(line_locations[1]) - origin)
    curvatures = []
    for point in line_locations[2:]:
        # second tangent vector
        next_point = (Vec3(line_locations[line_locations.index(point)]) - origin)
        # distance between two vectors
        curvature = (current_point.cross(next_point) / current_point.length()**3).length()
        # add it to list of curvatures
        curvatures.append(curvature)
        # advance the current point for next iteration
        current_point = point
    return curvatures

# find max velocity for a certain turn curvature
def find_max_turn_velocity(curvature):
    X = [0.0069,0.00398,0.00235,0.001375,0.0011,0.00088] # curvatures at which velocities shift
    Y = [0,500,1000,1500,1750,2300] # cooralating velocity values
    if X[-1] > curvature:
        return 2300
    # Defining the interpolation
    y_interp = interp1d(X, Y)
    # finding velocity from the interpolation function
    ending_velocity = y_interp(curvature)
    return ending_velocity

# find turn curvature given a velocity
def find_max_turn_curvature(velocity):
    X = [0,500,1000,1500,1750,2300] # velocities at which curvatures shift
    Y = [0.0069,0.00398,0.00235,0.001375,0.0011,0.00088] # cooralating curvature values
    # Defining the interpolation
    y_interp = interp1d(X, Y)
    # finding curvature from the interpolation function
    ending_velocity = y_interp(velocity)
    return ending_velocity 

# take in a location from the cars perspective, and return its absolute coordinates on the field
def relative_to_absolute(self, location):
    """Takes in a location from the cars perspective. Returns it's absolute coordinates"""
    absolute_location = self.location + location
    return absolute_location

# find the minimum time it will take to stop at the current speed
def time_to_stop(self):
    """Returns seconds it will take to come to a complete stop"""
    # braking reduces speed by 3500 units per second^2
    # cars current velocity
    current_speed = Vec3(self.car.physics.velocity).length()
    # using the braking values, and current speed, find how long (in seconds) it will take to come to a stop
    time_to_brake = current_speed/3500
    return time_to_brake


# find the minimum distance it will take to stop at the current speed
def distance_to_stop(self):
    """Returns how many game units needed to come to a stop at the current speed"""
    # braking distance = veloctiy / 2 * time
    braking_distance = Vec3(self.car.physics.velocity).length() / 2 * time_to_stop(self)
    return braking_distance

# find how time needed to reach a target ( does not yet factor in effects from turning)
# formula -----> time to target = time spent accelerating + (distance to target - distance to accelerate) / velocity
def time_to_target(self, target_speed=1410):
    """takes in speed as an integer and return how long it will take to arrive at the target in seconds"""
    time_spent_accelerating = time_to_accelerate(self, target_speed)
    acceleration_distance = distance_to_accelerate(self, target_speed)
    velocity = self.car_velocity.length()
    if acceleration_distance is not None:
        time_needed = time_spent_accelerating + (self.distance_to_target - acceleration_distance) / velocity
        return time_needed


    
# find how long it will take to accelerate to a certain speed (2.56 seconds until full speed)
# overshoots expected value from stop to full acceleration. prediction is 2.63 not 2.56 :(
def time_to_accelerate(self, target_speed = 1409.99):
    """Returns long it will take to accelerate to a speed in seconds"""
    velocity = Vec3(self.car_velocity).length()
    ticks = 0
    # until the velocity reaches the required speed
    while velocity < target_speed:
        # take our acceleration value
        acceleration = find_acceleration(velocity)
        # add the acceleration / 120 this tick to the velocity to get next ticks velocity
        # acceleration / 120 because we want the acceleration this tick, not per second
        velocity += acceleration / 120
        ticks += 1
    return ticks / 120 # divide by 120 to get how many seconds it will take to acheive the required speed
    

# find distance needed to accelerate to a specified speed (interpolation is inaccurate above a target speed of 1409)
def distance_to_accelerate(self, target_speed = 1409):
    """Given a speed, returns how many wold units it will take to accelerate to that speed"""
    velocity = Vec3(self.car_velocity).length()
    distance_traveled = 0
    # until we reach the target speed
    counter = 0
    while velocity < target_speed and velocity > 0:
        # find the acceleration this tick
        acceleration = find_acceleration(velocity) / 120
        # update the car velocity
        velocity += acceleration
        # increase the total distance driven by the velocity and acceleration this tick
        distance_traveled += velocity / 120
    return distance_traveled


# find acceleration value to be applied to the car per second
def find_acceleration(velocity, throttle=1):
    X = [0,1400,1410,2300] # speeds at which acceleration interpolation shifts
    Y = [1600,160,0,0] # cooralating acceleration values
    # Defining the interpolation
    y_interp = interp1d(X, Y)
    # finding the acceleration value from the interpolation function
    acceleration = y_interp(velocity)
    # adjust acceleration rate by throttle value
    acceleration *= throttle
    return acceleration




# @CHIP
# turn radius from velocity
def turn_radius(v):
    if v == 0:
        return 0
    return 1.0 / curvature(v)
# v is the magnitude of the velocity in the car's forward direction
def curvature(v):
    if 0.0 <= v < 500.0:
        return 0.006900 - 5.84e-6 * v
    if 500.0 <= v < 1000.0:
        return 0.005610 - 3.26e-6 * v
    if 1000.0 <= v < 1500.0:
        return 0.004300 - 1.95e-6 * v
    if 1500.0 <= v < 1750.0:
        return 0.003025 - 1.1e-6 * v
    if 1750.0 <= v < 2500.0:
        return 0.001800 - 4e-7 * v
    return 0.0

def display_visuals(self, target_location):
    self.renderer.draw_line_3d(self.car_contact_point, target_location, self.renderer.white())
    self.renderer.draw_string_3d(self.location, 1, 1, f'Speed: {self.car_velocity.length():.1f}', self.renderer.white())
    self.renderer.draw_rect_3d(target_location, 8, 8, True, self.renderer.cyan(), centered=True)
    # self.renderer.draw_rect_3d(self.front, 8, 8, True, self.renderer.red(), centered=True)
    
    self.renderer.draw_rect_3d(self.front_right_top_corner, 4, 4, True, self.renderer.yellow(), centered=True)
    self.renderer.draw_rect_3d(self.front_left_top_corner, 4, 4, True, self.renderer.yellow(), centered=True)
    self.renderer.draw_rect_3d(self.rear_right_top_corner, 4, 4, True, self.renderer.yellow(), centered=True)
    self.renderer.draw_rect_3d(self.rear_left_top_corner, 4, 4, True, self.renderer.yellow(), centered=True)

    self.renderer.draw_rect_3d(self.front_right_bottom_corner, 4, 4, True, self.renderer.yellow(), centered=True)
    self.renderer.draw_rect_3d(self.front_left_bottom_corner, 4, 4, True, self.renderer.yellow(), centered=True)
    self.renderer.draw_rect_3d(self.rear_right_bottom_corner, 4, 4, True, self.renderer.yellow(), centered=True)
    self.renderer.draw_rect_3d(self.rear_left_bottom_corner, 4, 4, True, self.renderer.yellow(), centered=True)
    
    self.renderer.draw_rect_3d(self.front_top, 4, 4, True, self.renderer.yellow(), centered=True)
    self.renderer.draw_rect_3d(self.rear_top, 4, 4, True, self.renderer.yellow(), centered=True)
    self.renderer.draw_rect_3d(self.front_bottom, 4, 4, True, self.renderer.yellow(), centered=True)
    self.renderer.draw_rect_3d(self.rear_bottom, 4, 4, True, self.renderer.yellow(), centered=True)
    
    self.renderer.draw_rect_3d(self.right_top, 4, 4, True, self.renderer.yellow(), centered=True)
    self.renderer.draw_rect_3d(self.left_top, 4, 4, True, self.renderer.yellow(), centered=True)
    self.renderer.draw_rect_3d(self.right_bottom, 4, 4, True, self.renderer.yellow(), centered=True)
    self.renderer.draw_rect_3d(self.left_bottom, 4, 4, True, self.renderer.yellow(), centered=True)
    
    # self.renderer.draw_rect_3d(self.ball_contact_point, 10, 10, True, self.renderer.blue(), centered=True)
    self.draw_ball_predictions(self.ball_prediction)

def draw_circle(self, location, radius=3000, step=1):
    # Playing with drawing geometry :)
    # circle formula:   r**2 = (x-h)**2 + (y-k)**2
    step = 50
    quarter_1_points, quarter_2_points, quarter_3_points, quarter_4_points = [],[],[],[]
    for x in range(0, radius+1, step): # adjusting by 1 for the way the range function works
        # the loop only get half of the y-values, need to invert the result to get the other half
        y = math.sqrt(radius**2 - x**2) 
        # putting each of the calculated points into its respective quadrant of the circle
        quarter_1_points.append(Vec3( x+location.x, y+location.y, location.z))
        quarter_2_points.append(Vec3( x+location.x,-y+location.y, location.z))
        quarter_3_points.append(Vec3(-x+location.x,-y+location.y, location.z))
        quarter_4_points.append(Vec3(-x+location.x, y+location.y, location.z))
    # inverting the lists, and combining them into one list of consecutive points
    points_to_plot = quarter_1_points + quarter_2_points[::-1] + quarter_3_points + quarter_4_points[::-1]
    # draw the line
    self.renderer.draw_polyline_3d(points_to_plot, self.renderer.cyan())
    return points_to_plot

def shooting_location(self):
    """Returns point on the ball opposite of the opponents goal"""
    ballHitPos = self.ball_location + Vec3(self.ball_location - Vec3(0,-5120, 53.50)).normalized() * 92.75
    # self.renderer.draw_rect_3d(ballHitPos, 10, 10, True, self.renderer.black(), centered=True)
    return ballHitPos