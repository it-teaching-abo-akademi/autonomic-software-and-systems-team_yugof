#!/usr/bin/env python

import glob
import os
import sys
from collections import deque
import math
from ai_knowledge import Status
import pandas as pd
import numpy as np
try:
    sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

# Executor is responsible for moving the vehicle around
# In this implementation it only needs to match the steering and speed so that we arrive at provided waypoints
# BONUS TODO: implement different speed limits so that planner would also provide speed target speed in addition to direction
class Executor(object):
  def __init__(self, knowledge, vehicle):
    self.vehicle = vehicle
    self.knowledge = knowledge
    self.target_pos = knowledge.get_location()
    self.flag_vit = 0
    
  #Update the executor at some intervals to steer the car in desired direction
  def update(self, time_elapsed):
    status = self.knowledge.get_status()
    #TODO: this needs to be able to handle
    if status == Status.DRIVING or status == Status.HEALING or status == Status.HEALING:
      dest = self.knowledge.get_current_destination()
      self.update_control(dest, [1], time_elapsed)
    if status == Status.CRASHED:
      self.vehicle.set_transform(self.knowledge.retrieve_data('transform_exit'))
      self.knowledge.update_status(Status.HEALING)  
  
  """#added
  def steer(self, destination):
  	basic_agent = ba.BasicAgent(self.vehicle)
  	_location = destination.transform.location
  	location = [_location.x,_location.y,_location.z]
  	basic_agent.set_destination(location)
  	while(basic_agent._local_planner._vehicle_status != ba.VehicleStatus.ARRIVED):
  		self.vehicle.apply_control(basic_agent.run_step(debug=False))
  		init_trans = vehicle.get_transform()
   	gap_trans_rot = 0
   	if direc == 0:
   		print("rotation droite voiture")
   		while gap_trans_rot < 90:
   			vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=1.0))
   			gap_trans_rot = init_trans.rotation.yaw - vehicle.get_transform().rotation.yaw
   		vehicle.apply_control(carla.VehicleControl(steer=1.0))
   	elif direc == 1:
   		while gap_trans_rot < 90:
   			vehicle.apply_control(carla.VehicleControl(steer=-1.0))
   			gap_trans_rot = init_trans.rotation.yaw - vehicle.get_transform().rotation.yaw
   	elif direc == 2:
   		print("forward voiture")
   		vehicle.apply_control(carla.VehicleControl(throttle=1.0))
   	else:
   		vehicle.apply_control(carla.VehicleControl(brake=1.0))"""

  # TODO: steer in the direction of destination and throttle or brake depending on how close we are to destination
  # TODO: Take into account that exiting the crash site could also be done in reverse, so there might need to be additional data passed between planner and executor, or there needs to be some way to tell this that it is ok to drive in reverse during HEALING and CRASHED states. An example is additional_vars, that could be a list with parameters that can tell us which things we can do (for example going in reverse)
  def update_control(self, destination, additional_vars, delta_time):
    control = self.vehicle.get_control()
    if destination == None:
      print("wait for destination")
      control.throttle = 0.0 
      control.steer = 0.0
      control.brake = 0.0
      control.hand_brake = True
    else:
      target_speed = self.knowledge.retrieve_data('target_speed')
      cur_speed = self.knowledge.retrieve_data('cur_speed')
      speed_delta = target_speed - cur_speed
      s1 = np.sign(speed_delta)
      control.steer = self.knowledge.retrieve_data('steer')
      s2 = np.sign(control.steer)
      
      if target_speed > 30 or self.flag_vit == 1:
        self.flag_vit = 1
        control.steer = control.steer*(math.pi-0.7)/(math.pi-1.2)
        control.throttle = min((speed_delta - s1*5) / target_speed + 0.7, 1.0) - abs(control.steer / 1.25)
      if cur_speed < 35:
        self.flag_vit = 0
        

      
      if target_speed > 0 and self.flag_vit == 0:
          control.throttle = min((speed_delta - s1*5) / target_speed + 0.7, 1.0) - abs(control.steer / 1.25)
          control.brake = 0.0
          control.hand_brake = False
          
      if target_speed == 0:
          control.throttle = 0.0
          control.brake = 1.0
          control.hand_brake = False
      """if self.knowledge.get_status() == Status.SAVE: Si pas d'obstacle devant ça vient coté ou derrière donc accélère, si obstacle devant freine
        control.throttle"""

    #print("control ok")
    return self.vehicle.apply_control(control)
    

    
    
        
        
    

# Planner is responsible for creating a plan for moving around
# In our case it creates a list of waypoints to follow so that vehicle arrives at destination
# Alternatively this can also provide a list of waypoints to try avoid crashing or 'uncrash' itself
class Planner(object):
  def __init__(self, knowledge):
    self.knowledge = knowledge
    self.path = deque([])
    self.path_crash = deque([])
    #self.knowledge.set_destination_changed_callback(self.destination_changed)
  # Create a map of waypoints to follow to the destination and save it
  def make_plan(self, source, destination):
    self.path = self.build_path(source,destination)
    self.update_plan()
    self.knowledge.update_destination(self.get_current_destination())
  
  # Function that is called at time intervals to update ai-state
  def update(self, time_elapsed):
    self.update_plan()
    self.knowledge.update_destination(self.get_current_destination())
    if self.knowledge.retrieve_data('light stop'):
      self.knowledge.update_data('target_speed',0)
    else:
      self.knowledge.update_data('target_speed',self.knowledge.retrieve_data('speed_limit'))
    print(self.knowledge.retrieve_data('target_speed')," & ",self.knowledge.retrieve_data('cur_speed'))

  """def destination_changed(self,new_destination):
    self.path = self.build_path(self.knowledge.retrieve_data('map').get_waypoint(self.knowledge.get_location()).transform,new_destination)
    self.update_plan()
    self.knowledge.update_destination(self.get_current_destination())"""
  
  #Update internal state to make sure that there are waypoints to follow and that we have not arrived yet
  def update_plan(self):
    if len(self.path) == 0:
      return
    
    if self.knowledge.get_status() == Status.SAVE:
      if self.knowledge.get_location().distance(self.knowledge.retrieve_data('safe_loc')) < 5:
        self.knowledge.update_status(Status.DRIVING)

    else:
      if self.knowledge.arrived_at(self.path[0]):
        self.path.popleft()
      if len(self.path) == 0:
        self.knowledge.update_status(Status.ARRIVED)
      else:
        self.knowledge.update_status(Status.DRIVING)

    

  #get current destination 
  def get_current_destination(self):
    status = self.knowledge.get_status()
    #if we are driving, then the current destination is next waypoint
    if status == Status.DRIVING or status == Status.HEALING:
      #TODO: Take into account traffic lights and other cars
      return self.path[0]
    if status == Status.ARRIVED:
      return self.knowledge.get_location()

    if status == Status.CRASHED:
      loc = self.knowledge.get_location()
      cur_wp = self.knowledge.retrieve_data('map').get_waypoint(loc)
      wp_list = cur_wp.next(5)
      dis = math.inf
      for wp in wp_list:
        if wp.transform.location.distance(self.path[0]) < dis:
          dis = wp.transform.location.distance(self.path[0])
          wp_exit = wp
      self.knowledge.update_data('transform_exit',wp.transform)
      return self.path[0]

    if status == Status.SAVE:
      return self.knowledge.retrieve_data('safe_loc')

    #otherwise destination is same as current position
    return self.knowledge.get_location()

  #TODO: Implementation
  def build_path(self,source, destination):
    self.path = deque([])
    #TODO: create path of waypoints from source to
    dest = carla.Location(destination.x,destination.y,destination.z)
    current_location = source.location
    current_waypoint = self.knowledge.retrieve_data('map').get_waypoint(current_location)
    while (current_waypoint.transform.location.distance(dest) > 7.5):
        list_next = current_waypoint.next(5)
        min = math.inf
        for waypoint in list_next:
            distance = waypoint.transform.location.distance(dest)
            if(distance < min):
                min = distance
                current_waypoint = waypoint
        self.path.append(current_waypoint.transform.location)
    self.path.append(dest)
    print(len(self.path),self.path[len(self.path)-1])
    return self.path
    """self.path = deque([])
    destination_wp = carla.Location(x=destination.x, y=destination.y, z=destination.z)

    def find_next(current, destination):
        md = math.inf
        nexts = current.next(5)
        for i in range(len(nexts)):
            point = nexts[i]
            dist = destination.distance(point.transform.location)
            if dist < md:
                md = dist
                idx = i
        if 'idx' not in locals():
            return current
        else:
            return nexts[idx]

    current_wp = self.knowledge.retrieve_data('map').get_waypoint(source.location)
    while True:
        next_point = find_next(current_wp, destination_wp)
        self.path.append(next_point.transform.location)
        if destination_wp.distance(next_point.transform.location) <= 7.5:
            break
        else:
            current_wp = next_point

    debug = True
    if debug:
        debug_markers_lifetime = 20.0
        self.knowledge.retrieve_data('world').debug.draw_string(source.location, "START", life_time=debug_markers_lifetime)
        for wp in self.path:
            self.knowledge.retrieve_data('world').debug.draw_string(wp, "o", life_time=debug_markers_lifetime)
        self.knowledge.retrieve_data('world').debug.draw_string(destination_wp, "END", life_time=debug_markers_lifetime)

    self.path.append(destination_wp)
    print(len(self.path),self.path[len(self.path)-1])
    return self.path"""
        
        


