#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2021 Ashwin Rajesh
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import sys

sys.path.append("/HDD/CARLA_0.9.11/PythonAPI/carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg")

import carla
import numpy as np
import time

def deg_to_rad(val):
    return val * np.pi / 180

class agent:
    # Constructor
    def __init__(self):
        self.client = carla.Client('localhost', 2000)
        self.world  = self.client.get_world()
        self.bp_lib = self.world.get_blueprint_library()

        self.vehicle = None
        self.actor_list = []

    # Spawn the vehicle randomly
    def spawn_vehicle(self, index=None):
        # Create blueprint and transform
        car_bp = np.random.choice(self.bp_lib.filter("vehicle.*.*"))
        print(" Vehicle model : %s %s"%(car_bp.id.split('.')[1], car_bp.id.split('.')[2]))
        if(index == None):    
            car_tf = np.random.choice(self.world.get_map().get_spawn_points())
            print(" Spawning vehicle at location : %d, %d, %d (random generation)"%(car_tf.location.x, car_tf.location.y, car_tf.location.z))
        else:
            car_tf = self.world.get_map().get_spawn_points()[index]
            print(" Spawning vehicle at location : %d, %d, %d (spawn point no. %d)"%(car_tf.location.x, car_tf.location.y, car_tf.location.z, index))

        # Spawn vehicle, append to actor list and set autopilot
        self.vehicle = self.world.spawn_actor(car_bp, car_tf)
        if(self.vehicle == None):
            print(" Vehicle could not be spawned")
            return
        self.actor_list.append(self.vehicle)
        self.vehicle.set_autopilot(True)

    # Spawn a camera
    def spawn_rgbcam(self, period=0.1):
        if(self.vehicle == None):
            print(" Error : spawn vehicle first")
            return

        # Define blueprint and transform
        bp = self.bp_lib.find("sensor.camera.rgb")
        
        tf = carla.Transform(carla.Location(0,0,2), carla.Rotation(0,0,0))

        self.rgbcam = self.world.spawn_actor(bp, tf, attach_to=self.vehicle)
        self.actor_list.append(self.rgbcam)

        self.rgbcam_time = 0
        self.rgbcam_per = period

        self.rgbcam_callbacks = []
        self.rgbcam.listen(self.rgbcam_listen)

    # Listener function for GNSS sensor
    def rgbcam_listen(self, data):
        if(data.timestamp - self.rgbcam_time < self.rgbcam_per):
            return

        self.rgbcam_time = data.timestamp
        self.rgbcam_data = data
        
        for c in self.rgbcam_callbacks:
            c(data)


    # Register a function to be called when gnss data is received
    def rgbcam_reg_callback(self, callback):
        self.rgbcam_callbacks.append(callback)

    # Spawn a GNSS sensor
    def spawn_gnss(self, period=0.1, std_dev=0.1):
        # Check if the vehicle was spawned
        if(self.vehicle == None):
            print(" Error : spawn car first")
            return

        # Define the blueprint and transform
        gnss_bp = self.bp_lib.find("sensor.other.gnss")
        gnss_bp.set_attribute('sensor_tick', str(period))
        gnss_bp.set_attribute('noise_lat_stddev', str(std_dev))
        gnss_bp.set_attribute('noise_lon_stddev', str(std_dev))
        gnss_tf = carla.Transform(carla.Location(0,0,0), carla.Rotation(0,0,0))
        
        # Spawning the sensor and appending to list
        self.gnss = self.world.spawn_actor(gnss_bp, gnss_tf, attach_to=self.vehicle)
        self.actor_list.append(self.gnss)
        # For timing
        self.gnss_time = 0
        self.gnss_per  = period
        # Register listen callback
        self.gnss_callbacks = []
        self.gnss.listen(self.gnss_listen)

    # Listener function for GNSS sensor
    def gnss_listen(self, data):
        if(data.timestamp - self.gnss_time < self.gnss_per):
            return
        self.gnss_time = data.timestamp
        self.gnss_data = data

        for c in self.gnss_callbacks:
            c(data)

    # Register a function to be called when gnss data is received
    def gnss_reg_callback(self, callback):
        self.gnss_callbacks.append(callback)
        
    # Destructor
    def __del__(self):
        for a in self.actor_list:
            a.destroy()
            self.actor_list.remove(a)

    # Destroy all actor
    def destroy_actors(self):
        for a in self.actor_list:
            a.destroy()
            self.actor_list.remove(a)

def get_img_from_data(data):
    array = np.frombuffer(data.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (data.height, data.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    
    return array.copy()
