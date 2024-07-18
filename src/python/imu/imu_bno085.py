import serial
import time
import math
import sys
import os
from threading import Lock

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(parent_dir)

import Adafruit_CircuitPython_BNO08x.adafruit_bno08x
import Adafruit_CircuitPython_BNO08x.adafruit_bno08x.uart

from imu import *


class AdafruitBNO085(IMU):
    _lock: Lock

    def __init__(self, dev='/dev/ttyAMA0'):
        self.is_setup = False
        self._lock = Lock()
        self._dev = dev

    def _setup(self):
        """Thread unsafe setup function.  Only used internally."""
        if not self.is_setup:
            try:
                uart = serial.Serial(self._dev, 3000000)
                self.sensor = Adafruit_CircuitPython_BNO08x.adafruit_bno08x.uart.BNO08X_UART(uart)

                log.info('Connected, now lets enable output')
                self.sensor.enable_feature(Adafruit_CircuitPython_BNO08x.adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
                self.sensor.enable_feature(Adafruit_CircuitPython_BNO08x.adafruit_bno08x.BNO_REPORT_GYROSCOPE)
                self.sensor.enable_feature(Adafruit_CircuitPython_BNO08x.adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
                self.sensor.enable_feature(Adafruit_CircuitPython_BNO08x.adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
                self.sensor.enable_feature(Adafruit_CircuitPython_BNO08x.adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION)
                self.sensor.enable_feature(Adafruit_CircuitPython_BNO08x.adafruit_bno08x.BNO_REPORT_GRAVITY)
                self.sensor.enable_feature(Adafruit_CircuitPython_BNO08x.adafruit_bno08x.BNO_REPORT_RAW_MAGNETOMETER)
                self.sensor.enable_feature(Adafruit_CircuitPython_BNO08x.adafruit_bno08x.BNO_REPORT_RAW_GYROSCOPE)
                self.sensor.enable_feature(Adafruit_CircuitPython_BNO08x.adafruit_bno08x.BNO_REPORT_RAW_ACCELEROMETER)

                self.is_setup = True
                self.calibration_status = None
                self.calibration_state = None
                # set the duration for checking calibration (seconds)
                self.wait_to_check_calibration_duration = 1
                # set the initial time for checking calibration
                self.check_calibration_time = time.time()

                #self.sensor.begin_calibration()

            except Exception as error:
                self.is_setup = False
                log.error(f"Adafruit BNO085 setup error: {error}")
            
    def setup(self):
        """Thread-safe setup function.  Call to setup the IMU."""
        with self._lock:
            self._setup()

    def _takeReading(self):
        """Thread unsafe takeReading function.  Only used internally."""
        if not self.is_setup:
            self._setup()

        try:
            quat_x, quat_y, quat_z, quat_w = self.sensor.quaternion
            quaternion = (quat_w, quat_x, quat_y, quat_z)
            linear_acceleration = self.sensor.linear_acceleration
            raw_mag_x, raw_mag_y, raw_mag_z = self.sensor.raw_magnetic
            raw_gyro_x, raw_gryo_y, raw_gyro_z = self.sensor.raw_gyro
            raw_accel_x, raw_accel_y, raw_accel_z = self.sensor.raw_acceleration
            cal_mag_x, cal_mag_y, cal_mag_z = self.sensor.magnetic
            cal_gyro_x, cal_gryo_y, cal_gyro_z = self.sensor.gyro
            cal_accel_x, cal_accel_y, cal_accel_z = self.sensor.acceleration

            gravity = self.sensor.gravity
            calibration_status = self.calibration_status
            calibration_state = self.calibration_state

            self.checkCalibration()
           
            if None in quaternion or None in linear_acceleration or None in gravity:
                log.warning("We received None data in the takeReading function")
                return None

            quaternion = Quaternion.from_tuple(quaternion)
            angular_velocity = Vector3(*self.sensor.gyro)
            orientation = quaternion.to_euler_angles()
            # even after consulting the docs, we're still off by 90 degrees!
            orientation.heading = (orientation.heading + 90) % 360
            linear_acceleration = Vector3(*linear_acceleration)
            linear_acceleration_world = quaternion.apply(linear_acceleration)
            gravity = Vector3(*gravity)

            raw_magnetometer = IMUSensorOutput(x=raw_mag_x, y=raw_mag_y, z=raw_mag_z)
            raw_gyroscope = IMUSensorOutput(x=raw_gyro_x, y=raw_gryo_y, z=raw_accel_z)
            raw_accelerometer = IMUSensorOutput(x=raw_accel_x, y=raw_accel_y, z=raw_accel_z)

            cal_magnetometer = IMUSensorOutput(x=cal_mag_x, y=cal_mag_y, z=cal_accel_z)
            cal_accelerometer = IMUSensorOutput(x=cal_accel_x, y=cal_accel_y, z=cal_accel_z)
            cal_gyroscope = IMUSensorOutput(x=cal_gyro_x, y=cal_gryo_y, z=cal_gyro_z)

            magnetometer_accuracy = self.sensor.get_magnetometer_accuracy()
            gyroscope_accuracy = self.sensor.get_gyroscope_accuracy()
            accelerometer_accuracy = self.sensor.get_accelerometer_accuracy()
            rotation_vector_accuracy = self.sensor.get_rotation_vector_accuracy()

            return IMUReading(orientation=orientation, 
                        linear_acceleration=linear_acceleration, 
                        linear_acceleration_world=linear_acceleration_world,
                        gravity=gravity,
                        calibration_status=calibration_status,
                        calibration_state=calibration_state,
                        quaternion=quaternion,
                        angular_velocity=angular_velocity,
                        raw_magnetometer=raw_magnetometer,
                        raw_gyroscope=raw_gyroscope,
                        raw_accelerometer=raw_accelerometer,
                        cal_magnetometer=cal_magnetometer,
                        cal_gyroscope=cal_gyroscope,
                        cal_accelerometer=cal_accelerometer,
                        magnetometer_accuracy=magnetometer_accuracy,
                        gyroscope_accuracy=gyroscope_accuracy,
                        accelerometer_accuracy=accelerometer_accuracy,
                        rotation_vector_accuracy=rotation_vector_accuracy
                        )

        except Exception as error:
            log.warning(f"Error trying to get data: {error}")

    def takeReading(self):
        """Thread-safe takeReading function.  Call to take a reading.


        Returns:
            IMUReading | None: An IMUReading object, if the reading was successful, otherwise None.
        """
        with self._lock:
            return self._takeReading()

    def startCalibration(self):
        self.calibration_state = CalibrationState.IN_PROGRESS
        self.calibration_good_at = None
        self.sensor.begin_calibration()

    def checkCalibration(self):
        if time.time() - self.check_calibration_time >= self.wait_to_check_calibration_duration:
            logging.debug("Checking Calibration")
            try:
                # set the calibration status to save when we are not querying a 
                # new calibration status
                self.calibration_status = self.sensor.calibration_status

                if self.calibration_state == CalibrationState.IN_PROGRESS:
                    logging.debug("Calibrating imu")
                    if not self.calibration_good_at and self.calibration_status > 2:
                        self.calibration_good_at = time.monotonic()
                        logging.debug("Record time of good calibration")
                    if self.calibration_good_at and (time.monotonic() - self.calibration_good_at > 5.0):
                        logging.debug("Good calibration has been achieved for over 5 seconds, saving calibration data")
                        self.sensor.save_calibration_data()
                        self.calibration_good_at = None
                        self.calibration_state = CalibrationState.COMPLETE
                        #self.sensor.disable_calibration()
            except Exception as error:
                log.warning("Error trying to get calibration status!")
            # reset the start time
            self.check_calibration_time = time.time()
        else:
            logging.debug("Waiting To Check Calibration")
