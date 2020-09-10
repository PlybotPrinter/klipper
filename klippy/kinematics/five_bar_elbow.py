# Five Bar Scara driven at elbows kinematics support
#
# Copyright (C) 2020 Pontus Borg <glpontus@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# TODO:
#   Add support for xoffset, yoffset and flipping of x and y
#   Fix so that homing uses linear stepper movements, Use a fake cartesian kinematics
#   Check other angle limits? such as head angle and angles by the body
#


import math, logging
import stepper, homing, mathutil, chelper

class FiveBarElbow:
    def __init__(self, toolhead, config):

        stepper_configs = [config.getsection('stepper_arm_left'),
                           config.getsection('stepper_arm_right')]

        rail_arm_left = stepper.PrinterRail(stepper_configs[0], 
                                            units_in_radians=True)
        rail_arm_right = stepper.PrinterRail(stepper_configs[1], 
                                             units_in_radians=True)

        rail_z = stepper.LookupMultiRail(config.getsection('stepper_z'))
        rail_z.setup_itersolve('cartesian_stepper_alloc', 'z')

        self.inner_distance = config.getfloat('inner_distance', above=0.)

        self.left_inner_arm = stepper_configs[0].getfloat('inner_arm_length', above=0.)
        self.left_outer_arm = stepper_configs[0].getfloat('outer_arm_length', above=0.)
        rail_arm_left.setup_itersolve('fivebarelbow_stepper_alloc', 'l',
                                      self.left_inner_arm, self.left_outer_arm,
                                      self.inner_distance)
        
        self.right_inner_arm = stepper_configs[1].getfloat('inner_arm_length', above=0.)
        self.right_outer_arm = stepper_configs[1].getfloat('outer_arm_length', above=0.)
        rail_arm_right.setup_itersolve('fivebarelbow_stepper_alloc', 'r',
                                      self.right_inner_arm, self.right_outer_arm,
                                      self.inner_distance)

        self.rails = [rail_arm_left, rail_arm_right, rail_z]

        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        config.get_printer().register_event_handler("stepper_enable:motor_off",
                                                    self._motor_off)
        
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)
        self.limit_z = (1.0, -1.0)
        self.homedXY = False

        logging.info("5-Bar elbow driven %.2f %.2f %.2f %.2f %.2f",
                     self.left_inner_arm, self.left_outer_arm, 
                     self.right_inner_arm, self.right_outer_arm,
                     self.inner_distance)
        
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def _distance(self, x0, y0, x1, y1):
        dx = x1-x0
        dy = y1-y0
        return math.sqrt(dx*dx + dy*dy)

    def _triangle_side(self, a, b, C):
        return math.sqrt((a*a) + (b*b) - (2*a*b*math.cos(C)))

    # Find the angle of the corner opposite to side c
    def _triangle_angle(self, a, b, c):
        # cosine rule
        cosC = (a*a + b*b - c*c) / ( 2*a*b)
        return math.acos(cosC)

    def _angles_to_position(self, left_angle, right_angle):
        # distance from attachement point to head
        left_d  = self._triangle_side(self.left_inner_arm,  self.left_outer_arm,  left_angle)
        right_d = self._triangle_side(self.right_inner_arm, self.right_outer_arm, right_angle)
        # distanmce from left attachement point
        dx = ((left_d*left_d) - (right_d*right_d) + (self.inner_distance*self.inner_distance)) / (2 * self.inner_distance)
        x = dx - self.inner_distance / 2
        y = math.sqrt((left_d*left_d) - (dx*dx))
        return [x,y]

    def _position_distances(self, x, y):
        left_d  = self._distance(-self.inner_distance/2,0, x, y)
        right_d = self._distance(self.inner_distance/2,0, x, y)
        return [left_d, right_d]

    def _position_to_angles(self, x, y):
        left_d, right_d  = self._position_distances(x, y)
        left_a  = self._triangle_angle(self.left_inner_arm, self.left_outer_arm, left_d)
        right_a  = self._triangle_angle(self.right_inner_arm, self.right_outer_arm, right_d)
        return [left_a, right_a, left_d, right_d]

    def calc_tag_position(self):
        # Motor pos -> caretesian coordinates
        left_angle  = self.rails[0].get_tag_position()
        right_angle = self.rails[1].get_tag_position()
        z_pos       = self.rails[2].get_tag_position()
        [x, y] = self._angles_to_position(left_angle, right_angle)
        #logging.info("5be calc_tag (%.2f, %.2f) -> (%.2f, %.2f, %.2f)", left_angle, right_angle, x, y, z_pos)
        return [x, y, z_pos]
    
    def set_position(self, newpos, homing_axes):
        #logging.info("5be set_position")
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
        if 0 in homing_axes and 1 in homing_axes:
            self.homedXY = True
        if 2 in homing_axes:
            self.limit_z = self.rails[2].get_range()
            logging.info("Set z limit %f %f", self.limit_z[0], self.limit_z[1])
    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limit_z = (1.0, -1.0)
    def home(self, homing_state):
        axes = homing_state.get_axes()
        logging.info("5be home %s", axes)
        if 0 in axes or 1 in axes: #  XY
            # Home left and right at the same time
            # klipper does homing with cartesian moves which is less than ideal
            #
            # Kevin21: On further thought, I think it should be possible to move all the steppers from the main toolhead motion queue (trapq) to a custom trapq
            #          during homing.  Then drip_move() could be moved from toolhead.py to homing.py.
            #          Then, it should be possible to swap in custom stepper_kinematics for steppers that need that.
            #          That might be a general improvement, as then we wouldn't have to worry about normal homing moves going through the kinematic check_moves()
            #
            # TODO: Swap over to a cartesian kinematics while homing to get linear stepper movements
            #
            homing_state.set_axes([0, 1])
            rails = [self.rails[0], self.rails[1]]
            [x,y] = self._angles_to_position(rails[0].get_homing_info().position_endstop, rails[1].get_homing_info().position_endstop)

            # Swap to linear kinematics
            # old_kin = stepper.set_kinematics(my_cartesian_sk)
            homepos  = [x, y, None, None]
            hil = rails[0].get_homing_info()
            if hil.positive_dir:
                forcepos = [0, 0, None, None]
            else:
                forcepos = [x, y, None, None]
            logging.info("5be home XY %s %s %s", rails, forcepos, homepos);

            homing_state.home_rails(rails, forcepos, homepos)
            logging.info("Homed XY")
            self.homedXY = True

            # Swap backto real kinematics, compute new real X Y and set it
            # new_pos = stepper.get_commanded_position() ; stepper.set_kinematics(old_kin) ; stepper.set_commanded_position(new_pos)
            # try: ...  except: stepper.set_kinematics(old_kin)
            
        if 2 in axes: # Z
            rail = self.rails[2]
            position_min, position_max = rail.get_range()
            hi = rail.get_homing_info()
            homepos = [None, None, None]
            homepos[2] = hi.position_endstop
            forcepos = list(homepos)
            if hi.positive_dir:
                forcepos[2] -= 1.5 * (hi.position_endstop - position_min)
            else:
                forcepos[2] += 1.5 * (position_max - hi.position_endstop)
            # Perform homing
            logging.info("5be home Z %s %s %s", [rail], forcepos, homepos); 
            homing_state.home_rails([rail], forcepos, homepos)
            
    def _motor_off(self, print_time):
        self.limit_z = (1.0, -1.0)

    def check_move(self, move):
        end_pos = move.end_pos

        # XY moves
        if move.axes_d[0] or move.axes_d[1]:
            xpos, ypos = end_pos[:2]
            logging.info("check_move %.2f %.2f", xpos, ypos)

            if not self.homedXY:
                raise move.move_error("Must home axis first")

            # Check that coordinate is in front
            if ypos < 0:
                raise move.move_error("Attempted move behind printer")

            # Make sure distance from left and right attachement point is no further away that inner+outer
            [left_d, right_d] = self._position_distances(xpos, ypos)
            if left_d > self.left_inner_arm+self.left_outer_arm or right_d > self.right_inner_arm+self.right_outer_arm:
                raise move.move_error("Attempted move outside reachable area (arm length)")

            # Check elbow angle limits
            [left_a, right_a, left_d, right_d] = self._position_to_angles(xpos, ypos)
            left_min_a, left_max_a = self.rails[0].get_range()
            if left_a < left_min_a or left_a > left_max_a:
                raise move.move_error("Attempted move left arm outside angle limits")

            right_min_a, right_max_a = self.rails[1].get_range()
            if right_a < right_min_a or right_a > right_max_a:
                raise move.move_error("Attempted move right arm outside angle limits")

            # TODO: Speed limits

        # Check Z
        if move.axes_d[2]:
            if end_pos[2] < self.limit_z[0] or end_pos[2] > self.limit_z[1]:
                if self.limit_z[0] > self.limit_z[1]:
                    raise homing.EndstopMoveError(
                        end_pos, "Must home axis first")
                raise homing.EndstopMoveError(end_pos)
            # Move with Z - update velocity and accel for slower Z axis
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(
                self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)

    def get_status(self, eventtime):
        xy_home = "" # TODO
        z_home = "z" if self.limit_z[0] <= self.limit_z[1] else ""
        return {'homed_axes': xy_home + z_home}

def load_kinematics(toolhead, config):
    return FiveBarElbow(toolhead, config)

