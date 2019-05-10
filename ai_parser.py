#!/usr/bin/env python

import glob
import os
import sys
import math
import time

try:
    sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import weakref
import carla
import numpy as np
from ai_knowledge import Status


# Monitor is responsible for reading the data from the sensors and telling it to the knowledge
# TODO: Implement other sensors (lidar and depth sensors mainly)
# TODO: Use carla API to read whether car is at traffic lights and their status, update it into knowledge
class Monitor(object):
  def __init__(self, knowledge,vehicle):
    self.vehicle = vehicle
    self.knowledge = knowledge
    weak_self = weakref.ref(self)
    self.collision = False
    self.iter = 0
    self.i = 0
    self.R = 0
    self.G = 0
    self.B = 0
    
    
    self.update(0)

    world = self.vehicle.get_world()
    self.knowledge.update_data('world', world)
    self.knowledge.update_data('map',world.get_map())
   
    
    # bp = world.get_blueprint_library().find('sensor.other.lane_detector')
    # self.lane_detector = world.spawn_actor(bp, carla.Transform(), attach_to=self.vehicle)
    # self.lane_detector.listen(lambda event: Monitor._on_invasion(weak_self, event))

    #Implementation of the lidar sensor and depth sensor
    # lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
    # lidar_bp.set_attribute('range','100')
    # lidar_bp.set_attribute('rotation_frequency','50.0')
    # self.lidar_ray_cast = world.spawn_actor(lidar_bp, carla.Transform(), attach_to=self.vehicle)
    # self.lidar_ray_cast.listen(lambda lidar: Monitor._ray_cast(weak_self, lidar))
    
    # depth_bp = world.get_blueprint_library().find('sensor.camera.depth')
    # depth_bp.set_attribute('image_size_x', '320')
    # depth_bp.set_attribute('image_size_y', '200')
    # depth_bp.set_attribute('fov', '110')
    # depth_bp.set_attribute('sensor_tick', '0.1')
    # depthCamera_transform = carla.Transform(carla.Location(x=-3,y=0, z=3))
    # depthCamera_transform.rotation.yaw += (180)
    # depthCamera_transform.rotation.pitch -= 35
    # self.depth_camera = world.spawn_actor(depth_bp, depthCamera_transform, attach_to=self.vehicle)
    # self.depth_camera.listen(lambda image: Monitor._depth_camera(weak_self,image))
    
    # obstacle_bp = world.get_blueprint_library().find('sensor.other.obstacle')
    # obstacle_bp.set_attribute('distance','100')
    # obstacle_bp.set_attribute('hit_radius','12')
    # obstacle_bp.set_attribute('only_dynamics','true')
    # obstacle_bp.set_attribute('debug_linetrace','true')
    """obstacle_transform = carla.Transform(carla.Location(x=1,y=1, z=3))
    obstacle_transform.rotation.yaw += (30+180)
    obstacle_transform.rotation.pitch -= 45"""
    #obstacle_transform = carla.Transform(carla.Location(x=0,y=0, z=0))
    #obstacle_transform.rotation.yaw += (30)
    #obstacle_transform.rotation.pitch -= 45
    # self.obs_detect = world.spawn_actor(obstacle_bp,carla.Transform() ,attach_to=self.vehicle)
    # self.obs_detect.listen(lambda event: Monitor._obs_actor(weak_self, event))
    
    collision_bp = world.get_blueprint_library().find('sensor.other.collision')
    self.collision = world.spawn_actor(collision_bp, carla.Transform(),attach_to=self.vehicle)
    self.collision.listen(lambda event: Monitor._on_collision(weak_self,event))
 
    #Implementation of rgb Camera
    # rgbCamera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    # rgbCamera_bp.set_attribute('image_size_x', '1920')
    # rgbCamera_bp.set_attribute('image_size_y', '1080')
    # rgbCamera_bp.set_attribute('fov', '110')
    # rgbCamera_bp.set_attribute('sensor_tick', '0.1')
    # rgbCamera_transform = carla.Transform(carla.Location(x=1,y=1, z=3))
    # rgbCamera_transform.rotation.yaw += (30+180)
    # rgbCamera_transform.rotation.pitch -= 45
    # self.rgbCamera = world.spawn_actor(rgbCamera_bp, rgbCamera_transform, attach_to=self.vehicle)
    # self.rgbCamera.listen(lambda image: self.knowledge.update_data('rgb_camera',image))

  #Function that is called at time intervals to update ai-state
  def update(self, time_elapsed):
    # Update the position of vehicle into knowledge
    self.knowledge.update_data('location', self.vehicle.get_location())
    self.knowledge.update_data('rotation', self.vehicle.get_transform().rotation)
    self.knowledge.update_data('velocity', self.vehicle.get_velocity())
    self.knowledge.update_data('speed_limit', self.vehicle.get_speed_limit())
    location = self.vehicle.get_transform().location
    destination = self.knowledge.get_current_destination()
    distance = location.distance(destination)
    if self.collision == True:
      self.iter += 1
      if self.iter > 20:
        if self.knowledge.retrieve_data('loc_deb_collision').distance(self.knowledge.get_location()) < 1:
          self.knowledge.update_status(Status.CRASHED)


    self.knowledge.update_data('light stop',self.vehicle.is_at_traffic_light() 
      and str(self.vehicle.get_traffic_light_state()) == 'Red'
      or str(self.vehicle.get_traffic_light_state()) == 'Yellow')
    

  # @staticmethod
  # def _on_invasion(weak_self, event):
    # self = weak_self()
    # if not self:
      # return
    # put in lane invasion the list of lane crossed
    # self.knowledge.update_data('lane_invasion',event.crossed_lane_markings)
    # print("Lane crossed")

  # @staticmethod
  # def _ray_cast(weak_self, lidar):
    # self = weak_self()
    # if not self:
      # return
    # self.knowledge.update_data('ray_cast_points',lidar)
    # print("test1")
    '''
    start = time.time()
    for loc in lidar:
      #print("taille lidar",len(lidar))
      #print("location in z = ",loc.z)
      cur_loc = self.knowledge.get_location()
      #print("distance entre points et moi = ",self.knowledge.distance(cur_loc,loc))
      if True:
        if self.knowledge.distance(cur_loc,loc) < 1.5:
          #print("problème")
          self.knowledge.update_status(Status.SAVE)
          self.knowledge.update_data('safe_loc',carla.Location(6*cur_loc.x-loc.x,6*cur_loc.y-loc.y))
        elif self.knowledge.get_status() == Status.SAVE:
          self.knowledge.update_status(Status.DRIVING)
    print("exec time loop",(time.time() - start))'''


  @staticmethod
  def _on_collision(weak_self,event):
    self = weak_self()
    if not self:
      return
    self.knowledge.update_data('collision',event.other_actor)
    self.collision = True
    self.knowledge.update_data('loc_deb_collision',self.knowledge.get_location())

  # @staticmethod 
  # def _obs_actor(weak_self,event):
    # self = weak_self()
    # if not self:
      # return
    #print("Actor = ", event.other_actor.type_id,"; Distance to self.vehicle = ",self.knowledge.distance(self.knowledge.get_location(),event.other_actor.get_location()))
    # if str(event.other_actor.type_id) != "static.vegetation":
      # print(event.other_actor.type_id)

  # @staticmethod

  # def _depth_camera(weak_self,image):
    # self = weak_self()
    # if not self:
      # return
    # self.knowledge.update_data('depth_camera', image)
    
    # for data in image.raw_data:
      # if self.i == 0:
        # self.R = data
        # self.i += 1
      # elif self.i == 1:
        # self.G = data
        # self.i += 1
      # else:
        # self.B = data
        # self.i = 0
        # normalized = (self.R + self.G * 256 + self.B * 256 * 256) / (256 * 256 * 256 - 1)
        # in_meters = 1000 * normalized
        # print('in_meters is',in_meters )
    # print('boucle')
    






# Analyser is responsible for parsing all the data that the knowledge has received from Monitor and turning it into something usable
# TODO: During the update step parse the data inside knowledge into information that could be used by planner to plan the route
class Analyser(object):
  def __init__(self, knowledge):
    self.knowledge = knowledge

  
  #Function that is called at time intervals to update ai-state
  
  def update(self, time_elapsed):
      self.knowledge.update_data('steer' , self.calculateSteer())
      self.knowledge.update_data('cur_speed' ,self.velocity2speed(self.knowledge.retrieve_data('velocity')))

  def calculateSteer(self):
      cur_location = self.knowledge.get_location()
      dest_location = self.knowledge.get_current_destination()
      cur_rad = math.radians(self.knowledge.get_rotation())
      tar_rad = math.atan2(dest_location.y - cur_location.y,dest_location.x - cur_location.x)
      delta_rad = tar_rad - cur_rad
      
      if delta_rad > math.pi:
          delta_rad = delta_rad - (2 * math.pi)
      
      if delta_rad < -1*(math.pi):
          delta_rad = delta_rad + (2 * math.pi)
      
      
      return (delta_rad/(math.pi-0.7))


  def velocity2speed(self,velocity):
      return math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2) * 3.6
      
      
