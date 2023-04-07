# Axis Twist Compensation

# Copyright (C) 2022  Jeremy Tan <jeremytkw98@gmail.com>
# This file may be distributed under the terms of the GNU GPLv3 license.

"""
[axis_twist_compensation]
horizontal_move_z: 10
speed: 50
start_x: 0 ; nozzle's x coordinate at the start of the calibration ! required
end_x: 200 ; nozzle's x coordinate at the end of the calibration ! required
y: 100 ; nozzle's y coordinate during the calibration ! required
"""

import logging
import math
from . import manual_probe as ManualProbe, bed_mesh as BedMesh

DEFAULT_N_POINTS = 3


class Config:
    DEFAULT_SPEED = 50.
    DEFAULT_HORIZONTAL_MOVE_Z = 10.
    REQUIRED = True
    OPTIONAL = False
    CONFIG_OPTIONS = {
        'horizontal_move_z': (float, OPTIONAL, DEFAULT_HORIZONTAL_MOVE_Z),
        'speed': (float, OPTIONAL, DEFAULT_SPEED),
        'start_x': (float, REQUIRED, None),
        'end_x': (float, REQUIRED, None),
        'y': (float, REQUIRED, None)
    }

    def __init__(self, z_compensations, recommended_z_offset):
        self.z_compensations = z_compensations
        self.recommended_z_offset = recommended_z_offset

    @staticmethod
    def load_from_config_data(config):
        return Config(
            Helpers.parse_comma_separated_floats(
                config.get('z_compensations', default="")),
            config.getfloat('recommended_z_offset', default=0.0),
        )

    def save_to_config(self, name, configdata):
        values_as_str = ', '.join([Helpers.format_float_to_n_decimals(x)
                                   for x in self.z_compensations])
        configdata.set(name, 'z_compensations', values_as_str)
        configdata.set(name, 'recommended_z_offset',
                       Helpers.format_float_to_n_decimals(
                           self.recommended_z_offset))


class AxisTwistCompensation:
    def __init__(self, config):
        # get printer
        self.printer = config.get_printer()

        # get values from [axis_twist_compensation] section in printer .cfg
        for config_key, \
            (config_type, required, default) in Config.CONFIG_OPTIONS.items():
            value = None
            if config_type == float:
                value = config.getfloat(config_key, default)
            else:
                value = config.get(config_key, default)
            if required and value is None:
                raise config.error(
                    "Missing required config option for section [{}]: {}"
                    .format(config.get_name(), config_key))
            setattr(self, config_key, value)

        # setup persistent storage
        self.configmgr = ConfigManager(config, self)

        # setup calibrater
        calibrater_config = {
            'horizontal_move_z': self.horizontal_move_z
                if hasattr(self, 'horizontal_move_z') else None,
            'speed': self.speed if hasattr(self, 'speed') else None,
            'start_x': self.start_x if hasattr(self, 'start_x') else None,
            'end_x': self.end_x if hasattr(self, 'end_x') else None,
            'y': self.y if hasattr(self, 'y') else None
        }
        self.calibrater = Calibrater(
            config, self.configmgr, calibrater_config)

    def get_z_compensation_value(self, x_coord):
        current_config = self.configmgr.get_config()
        z_compensations = current_config.z_compensations
        if not z_compensations:
            return 0
        n_points = len(z_compensations)
        spacing = (self.end_x - self.start_x) / (n_points - 1)
        interpolate_t = (x_coord - self.start_x) / spacing
        interpolate_i = int(math.floor(interpolate_t))
        interpolate_i = BedMesh.constrain(interpolate_i, 0, n_points - 2)
        interpolate_t -= interpolate_i
        interpolated_z_compensation = BedMesh.lerp(
            interpolate_t, z_compensations[interpolate_i],
            z_compensations[interpolate_i + 1])
        return interpolated_z_compensation

    def _get_mesh_point_x_coord(self, col_index, mesh):
        # returns the x coordinate of the given column index
        # in the probed matrix
        x_min = mesh.mesh_x_min
        x_range = mesh.mesh_x_max - mesh.mesh_x_min
        x_step = x_range / (len(mesh.probed_matrix[0]) - 1)
        return x_min + col_index * x_step


class Calibrater:
    def __init__(self, config, configmgr, calibrater_config):
        # setup self attributes
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.configmgr = configmgr
        self.probe = None
        # probe settings are set to none, until they are available
        self.lift_speed, self.probe_x_offset, self.probe_y_offset, _ = \
            None, None, None, None
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect(config))
        self.speed = calibrater_config['speed']
        self.horizontal_move_z = calibrater_config['horizontal_move_z']
        self.start_point = (
            calibrater_config['start_x'], calibrater_config['y'])
        self.end_point = (calibrater_config['end_x'], calibrater_config['y'])
        self.results = None
        self.current_point_index = None
        self.gcmd = None

        # register gcode handlers
        self._register_gcode_handlers()

    def _handle_connect(self, config):
        # gets probe settings when they are available
        def callback():
            self.probe = self.printer.lookup_object('probe', None)
            if (self.probe is None):
                raise config.error(
                    "AXIS_TWIST_COMPENSATION requires [probe] to be defined")
            self.lift_speed = self.probe.get_lift_speed()
            self.probe_x_offset, self.probe_y_offset, _ = \
                self.probe.get_offsets()
        return callback

    def _register_gcode_handlers(self):
        # register gcode handlers
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command(
            'AXIS_TWIST_COMPENSATION_CALIBRATE',
            self.cmd_AXIS_TWIST_COMPENSATION_CALIBRATE,
            desc=self.cmd_AXIS_TWIST_COMPENSATION_CALIBRATE_help)

    cmd_AXIS_TWIST_COMPENSATION_CALIBRATE_help = """
    Performs the x twist calibration wizard
    Measure z probe offset at n points along the x axis,
    and calculate x twist compensation
    """

    def cmd_AXIS_TWIST_COMPENSATION_CALIBRATE(self, gcmd):
        self.gcmd = gcmd
        n_points = gcmd.get_int('N_POINTS', DEFAULT_N_POINTS)

        # check for valid n_points
        if n_points is None or n_points < 3:
            raise self.gcmd.error(
                "N_POINTS to probe must be at least 3")

        # clear the current config
        self.configmgr.clear_config()

        # calculate some values
        x_range = self.end_point[0] - self.start_point[0]
        interval_dist = x_range / (n_points - 1)
        nozzle_points = self._calculate_nozzle_points(n_points, interval_dist)
        probe_points = self._calculate_probe_points(
            nozzle_points, self.probe_x_offset, self.probe_y_offset)

        # verify no other manual probe is in progress
        ManualProbe.verify_no_manual_probe(self.printer)

        # begin calibration
        self.current_point_index = 0
        self.results = []
        self._calibration(probe_points, nozzle_points, interval_dist)

    def _calculate_nozzle_points(self, n_points, interval_dist):
        # calculate the points to put the probe at, returned as a list of tuples
        nozzle_points = []
        for i in range(n_points):
            x = self.start_point[0] + i * interval_dist
            y = self.start_point[1]
            nozzle_points.append((x, y))
        return nozzle_points

    def _calculate_probe_points(self, nozzle_points,
        probe_x_offset, probe_y_offset):
        # calculate the points to put the nozzle at
        # returned as a list of tuples
        probe_points = []
        for point in nozzle_points:
            x = point[0] - probe_x_offset
            y = point[1] - probe_y_offset
            probe_points.append((x, y))
        return probe_points

    def _move_helper(self, target_coordinates, override_speed=None):
        # pad target coordinates
        target_coordinates = \
            (target_coordinates[0], target_coordinates[1], None) \
            if len(target_coordinates) == 2 else target_coordinates
        toolhead = self.printer.lookup_object('toolhead')
        speed = self.speed if target_coordinates[2] == None else self.lift_speed
        speed = override_speed if override_speed is not None else speed
        toolhead.manual_move(target_coordinates, speed)

    def _calibration(self, probe_points, nozzle_points, interval):
        # begin the calibration process
        self.gcmd.respond_info("AXIS_TWIST_COMPENSATION_CALIBRATE: "
                               "Probing point %d of %d" % (
                                   self.current_point_index + 1,
                                   len(probe_points)))

        # horizontal_move_z (to prevent probe trigger or hitting bed)
        self._move_helper((None, None, self.horizontal_move_z))

        # move to point to probe
        self._move_helper((probe_points[self.current_point_index][0],
                           probe_points[self.current_point_index][1], None))

        # probe the point
        self.current_measured_z = self.probe.run_probe(self.gcmd)[2]

        # horizontal_move_z (to prevent probe trigger or hitting bed)
        self._move_helper((None, None, self.horizontal_move_z))

        # move the nozzle over the probe point
        self._move_helper((nozzle_points[self.current_point_index]))

        # start the manual (nozzle) probe
        ManualProbe.ManualProbeHelper(
            self.printer, self.gcmd,
            self._manual_probe_callback_factory(
                probe_points, nozzle_points, interval))

    def _manual_probe_callback_factory(self, probe_points,
        nozzle_points, interval):
        # returns a callback function for the manual probe
        is_end = self.current_point_index == len(probe_points) - 1

        def callback(kin_pos):
            if kin_pos is None:
                # probe was cancelled
                self.gcmd.respond_info(
                    "AXIS_TWIST_COMPENSATION_CALIBRATE: Probe cancelled, "
                    "calibration aborted")
                return
            z_offset = self.current_measured_z - kin_pos[2]
            self.results.append(z_offset)
            if is_end:
                # end of calibration
                self._finalize_calibration()
            else:
                # move to next point
                self.current_point_index += 1
                self._calibration(probe_points, nozzle_points, interval)
        return callback

    def _finalize_calibration(self):
        # finalize the calibration process
        # calculate average of results
        avg = sum(self.results) / len(self.results)
        # subtract average from each result
        # so that they are independent of z_offset
        self.results = [avg - x for x in self.results]
        # save the config
        self.configmgr.set_config(Config(self.results, avg))
        # recommend z offset to user
        self.gcmd.respond_info(
            "AXIS_TWIST_COMPENSATION_CALIBRATE: Calibration complete, "
            "offsets: %s, recommended z_offset: %f"
            % (self.results, avg))


class ConfigManager:
    def __init__(self, config, axis_twist_compensation):
        # setup self attributes
        self.name = config.get_name()
        self.printer = config.get_printer()
        self.axis_twist_compensation = axis_twist_compensation
        self.gcode = self.printer.lookup_object('gcode')

        # fetch the stored config
        self._fetch_stored_config(config)

        # register gcode handlers
        self._register_gcode_handlers()

    def get_config(self):
        return self.config

    def set_config(self, config):
        self.config = config
        self._save_config()

    def _fetch_stored_config(self, config):
        stored_config = config.getsection(self.name)
        self.config = Config.load_from_config_data(stored_config)

    def _register_gcode_handlers(self):
        # register gcode handlers
        self.gcode.register_command(
            'AXIS_TWIST_COMPENSATION_CLEAR',
            self.cmd_AXIS_TWIST_COMPENSATION_CLEAR,
            desc=self.cmd_AXIS_TWIST_COMPENSATION_CLEAR_help)

    def clear_config(self):
        self.config = Config([], 0.0)

    def _save_config(self):
        configfile = self.printer.lookup_object('configfile')
        self.config.save_to_config(self.name, configfile)
        self.gcode.respond_info(
            "AXIS_TWIST_COMPENSATION state has been saved\n"
            "for the current session.  The SAVE_CONFIG command will\n"
            "update the printer config file and restart the printer.")

    cmd_AXIS_TWIST_COMPENSATION_CLEAR_help = \
        "Clears the active axis twist compensation"

    def cmd_AXIS_TWIST_COMPENSATION_CLEAR(self, gcmd):
        # clears the active mesh
        self.clear_config()


class Helpers:
    @staticmethod
    def format_float_to_n_decimals(raw_float, n=6):
        # format float to n decimals, defaults to 6
        return "{:.{}f}".format(raw_float, n)

    @staticmethod
    def parse_comma_separated_floats(comma_separated_floats):
        if not comma_separated_floats:
            return []
        # parse comma separated floats into list of floats
        return [float(value) for value in comma_separated_floats.split(', ')]

# klipper's entry point using [axis_twist_compensation] section in printer.cfg

def load_config(config):
    return AxisTwistCompensation(config)
