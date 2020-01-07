# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2019 Bitcraze AB
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example of a swarm using the High level commander.

The swarm takes off and flies a synchronous square shape before landing.
The trajectories are relative to the starting positions and the Crazyfles can
be at any position on the floor when the script is started.

This example is intended to work with any absolute positioning system.
It aims at documenting how to use the High Level Commander together with
the Swarm class.
"""
import time

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    wait_for_position_estimator(scf)


def activate_high_level_commander(scf):
    scf.cf.param.set_value('commander.enHighLevel', '1')


def activate_mellinger_controller(scf, use_mellinger):
    controller = 1
    if use_mellinger:
        controller = 2
    scf.cf.param.set_value('stabilizer.controller', controller)


def run_shared_sequence(scf, drone_params):
    activate_mellinger_controller(scf, False)

    box_size = .8
    flight_time = 3
    
    count = 0
    
    x_points = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    y_points = [-1,-0.88778,-0.7753,-0.66303,-0.55171,-0.44262,-0.28776,-0.19512,-0.11623,-0.03448,0,0.03448,0.11623,0.19512,0.28776,0.44262,0.55171,0.66303,0.7753,0.88778,1]
    z_points = [1,0.96498,0.92298,0.8724,0.81134,0.7377,0.5995,0.4878,0.36322,0.17241,0,0.17241,0.36322,0.4878,0.5995,0.7377,0.81134,0.8724,0.92298,0.96498,1]
  

    commander = scf.cf.high_level_commander

    commander.takeoff(1.0, 2.0)
    time.sleep(3)
    
    commander.go_to(drone_params['start_x'], drone_params['start_y'], 1.5, 0, 5, relative=False)
    scf.cf.param.set_value('ring.effect', 7)
    scf.cf.param.set_value('ring.solidBlue', drone_params['b'])
    scf.cf.param.set_value('ring.solidRed', drone_params['r'])
    scf.cf.param.set_value('ring.headlightEnable', 1)
    time.sleep(5)
    commander.go_to(drone_params['start_x'], drone_params['start_y'], 1, 0, 5, relative=False)
    scf.cf.param.set_value('ring.effect', 6)
    scf.cf.param.set_value('ring.headlightEnable', 0)
    time.sleep(5)
    scf.cf.param.set_value('ring.headlightEnable', 1)
    commander.go_to(drone_params['start_x'], drone_params['start_y'], 1.5, 0, 5, relative=False)
    time.sleep(5)
    
    scf.cf.param.set_value('ring.headlightEnable', 0)
    time.sleep(0.2)
    scf.cf.param.set_value('ring.headlightEnable', 1)
    time.sleep(0.2)
    scf.cf.param.set_value('ring.headlightEnable', 0)
    time.sleep(0.2)
    scf.cf.param.set_value('ring.headlightEnable', 1)
    time.sleep(0.2)
    scf.cf.param.set_value('ring.headlightEnable', 0)
    
    commander.go_to(box_size, 0, drone_params['box'], 0, flight_time, relative=True)
    scf.cf.param.set_value('ring.effect', 7)
    scf.cf.param.set_value('ring.solidBlue', drone_params['b'])
    scf.cf.param.set_value('ring.solidRed', drone_params['r'])
    scf.cf.param.set_value('ring.headlightEnable', 1)
    time.sleep(flight_time)
  
    commander.go_to(0, box_size, -drone_params['box'], 0, drone_params['flytime'], relative=True)
    time.sleep(5)

    commander.go_to(-box_size, 0, drone_params['box'], 0, 10/drone_params['flytime'], relative=True)
    time.sleep(flight_time)

    commander.go_to(0, -box_size, -0.7, 0, flight_time, relative=True)
    scf.cf.param.set_value('ring.effect', 6)
    scf.cf.param.set_value('ring.headlightEnable', 0)
    time.sleep(flight_time)
    
    commander.go_to(0, 0, box_size/3, 0, flight_time, relative=True)
    scf.cf.param.set_value('ring.effect', 6)
    scf.cf.param.set_value('ring.headlightEnable', 1)
    time.sleep(flight_time)
    
    commander.go_to(drone_params['start_x'], drone_params['start_y'], 1, 0, 5, relative=False)
    scf.cf.param.set_value('ring.effect', 0)
    scf.cf.param.set_value('ring.headlightEnable', 1)
    time.sleep(5)
    
    print("starting loop sequence")

    while count <= len(z_points)+1: 
        if count == len(z_points):
            break
           
        commander.go_to(x_points[count], x_points[count], z_points[count], 0, 2, relative=True)
        scf.cf.param.set_value('ring.effect', 7)
        scf.cf.param.set_value('ring.solidBlue', drone_params['b'])
        scf.cf.param.set_value('ring.solidRed', drone_params['r'])
        scf.cf.param.set_value('ring.headlightEnable', 1)
        count += 1
        time.sleep(2)
        
    print("finished loop sequence")
    commander.go_to(drone_params['start_x'], drone_params['start_y'], 1, 0, 5, relative=False)
    scf.cf.param.set_value('ring.effect', 0)
    scf.cf.param.set_value('ring.headlightEnable', 0)
    time.sleep(5)

    commander.land(0.0, 2.0)
    time.sleep(2)

    commander.stop()




URI0 = 'radio://0/80/2M/E7E7E7E705'
URI1 = 'radio://0/80/2M/E7E7E7E706'

params0 = {'r': 255, 'b': 0, 'start_x': 0.5, 'start_y': .7, 'flytime': 3, 'box': 0.5}
params1 = {'r': 0, 'b': 255, 'start_x': .8, 'start_y': 1.0, 'flytime': 5, 'box': 0.5}

uris = {
    URI0,
    URI1
}

params = {
    URI0: [params0],
    URI1: [params1]
}



if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(activate_high_level_commander)
        swarm.parallel_safe(reset_estimator)
        swarm.parallel_safe(run_shared_sequence, args_dict=params)
