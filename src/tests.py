# v = d / t
from tools import *
from util.vec import Vec3
import copy
from time import sleep
from util.orientation import Orientation, relative_location



""" ACCELERATION PREDICTION TESTING """
# testing the time_to_accelerate functions accuracy
def test_acceleration_prediction(self, packet):
    if self.triggered == False:
        self.time_triggered = copy.deepcopy(packet.game_info.seconds_elapsed)
        self.expected_time = time_to_accelerate(self)
        self.triggered = True

    elif self.car_velocity.length() > 1409.99:
        print(f"Time expected: {self.expected_time}")
        print(f"Time taken: {packet.game_info.seconds_elapsed - self.time_triggered}")
        sleep(10)



""" BRAKING PREDICTION TESTING """
# paste the below commented out code below into the main loop to use test_braking_accuracy()
# if self.test_completed == False:
#             self = test_braking_accuracy(self, packet)
#         else:
#             sleep(5)
#             self.test_completed = False
#             self.throttle = 1
#         distance_to_stop(self)
# test accuracy by braking for the amount of seconds calculated above
def test_braking_prediction(self, packet):
    # if car speed reaches 1200
    if Vec3(self.car.physics.velocity).length() >= 1300 and self.triggered == False:
        # set the testing flag
        self.triggered = True
        # get the braking start time
        self.time_triggered = copy.deepcopy(packet.game_info.seconds_elapsed)
        # get how long we expect to brake
        self.time_to_brake = time_to_stop(self)
        # get current location of where we begin stopping
        self.test_start_location = Vec3(self.car.physics.location)
        # get how far we expect to travel before stopping
        self.expected_distance = distance_to_stop(self)

    # if the test has begun
    if self.triggered == True:
        # if the cars velocity is decreasing (has not yet stopped)
        if Vec3(self.car.physics.velocity).length() < self.testing_velocity:
            # get the car veloctiy and game_seconds of this tick
            self.testing_velocity = Vec3(self.car.physics.velocity).length()
            self.previous_tick_time = copy.deepcopy(packet.game_info.seconds_elapsed)
        # if cars velocity begins increasing (we came to a stop and have began moving backwards)
        else:
            # find how long it took to stop using the previous game tick data (the last time we decreased velocity)
            self.actual_braking_time = packet.game_info.seconds_elapsed - self.time_triggered

        print("Test Running")
        # apply the brake for the expected amount of time
        if self.time_triggered + self.time_to_brake > packet.game_info.seconds_elapsed:
            self.throttle = -1
        else:
            # get what time we stopped
            self.time_stopped = copy.deepcopy(packet.game_info.seconds_elapsed)
            # get current location of where we complete stopping
            ending_location = Vec3(self.car.physics.location)
            actual_distance = self.test_start_location.dist(ending_location)
            print(f"Expected Stop Distance: {self.expected_distance} \nActual Stop Distance {actual_distance}")
            print(f"Expected Stop Time: {self.time_to_brake} \nActual Stop Time: {self.actual_braking_time}")
            self.throttle = 0
            self.triggered = False
            self.test_completed = True
    return self


# Overall goal below is to be able to find the turning value and speed required
#   to make an even smooth turn to a location

# worst case scenario:
# 1. have car start with turn set to 1 or -1 (direction doesnt matter)
# 2. slowly increase speed from zero to 1400
# 3. record each speed and the resulting diameter as a dictionary value pair.
#       Diameter can be found by measuring the min/max x and y values during a 360
#       turn of consistant speed. maxX-minX = Diameter.  Diameter/2 = turn radius
# Example:
#   radius_lookup_table = {velocity : turn_radius}
#   turn_speed_table = {turn_radius : velocity}

# if feeling brave afterwards, plot the data to a chart and write an interpolation (piecewise) function 
# that can calculate curves to match the charted data. After that the lookup tables will no longer be needed

""" Finding turning radius for every throttle value in .001 increments """
#function to the the above trouble
def gather_turning_data(self, packet):
    if self.throttle <= 1:
        self.game_ticks += 1
        # set turning to max
        steering = 1
        if self.triggered == False:
            self.time_triggered = copy.deepcopy(packet.game_info.seconds_elapsed)
            # drive in a circle until velocity is stable then record diameter
            if self.turn_max == None:
                self.turn_max = self.location.x
            if self.turn_min == None:
                self.turn_min = self.location.x
            self.triggered = True
        else:
            if self.location.x > self.turn_max:
                self.turn_max = self.location.x
            if self.location.x < self.turn_min:
                self.turn_min = self.location.x
            # every 3 seconds record the turn radius
            if self.game_ticks >= 3 * 120:
                print(f'---Saving Turn Radius For Speed: {self.throttle}---')
                self.turn_data
                self.turn_data.append(self.turn_max-self.turn_min)
                self.triggered = False
                self.test_counter += 1
                self.turn_max = None
                self.turn_min = None
                self.game_ticks = 0
            # every 10 records saved
            if self.test_counter >= 15:
                # get the 10 record average, throwing out the first 5 indexes because of possible lingering acceleration
                ten_run_average = sum(self.turn_data[5:])/10
                self.ten_run_averages.append(sum(self.turn_data[5:])/10)
                print(f"Ten run average at {self.throttle} throttle: {ten_run_average}")
                self.turn_data = []
                self.triggered = False
                self.test_counter = 0
                self.turn_max = None
                self.turn_min = None
                self.throttle += .001
                print(f"increasing throttle to: {self.throttle}")
                pickle.dump(self.ten_run_averages, open('.000-.249.pkl', 'wb'))
    return steering, self.throttle



def arc_to_target(self):
    
    # if the test has not began
    if self.start_location is None:
        # set static start point of the sin() wave
        self.start_location = self.car_contact_point
        # end location
        self.destination_location = self.ball_location
        # phase shift factor (distance between where the wave crosses the x-axis)
        self.phase_shift = abs(self.destination_location.x - self.start_location.x)
        self.amplitude = abs(self.ball_location.y - self.start_location.y ) 
        self.wave_direction = -1
        self.step_interval = 3
        self.y_axis_shift = self.car_contact_point.y
        self.x_axis_shift = math.pi*1/2
        if self.start_location.x < self.destination_location.x:
            self.step_interval *= -1
        if self.start_location.y > self.destination_location.y:
            self.y_axis_shift *= -1

    #sin wave from starting location to the destination
    location_y = self.wave_direction * self.amplitude * (math.sin(((2/self.phase_shift)*self.start_location.x*math.pi)/2 - self.x_axis_shift)) + self.y_axis_shift
    location = Vec3(self.start_location.x, location_y, 0)
    self.start_location.x += self.step_interval * self.wave_direction


    if abs(self.start_location.x) == abs(self.destination_location.x):
        self.start_location = None
    if location.y < -5000:
        location.y = -5000
    if location.y > 5000:
        location.y = 5000
    if location.x < -4000:
        location.x = -4000
    if location.x > 4000:
        location.x = 4000

    return location