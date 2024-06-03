import rospy

from std_srvs.srv import Empty
from nortek_nucleus_dvl.srv import (
    ConnectTcp,
    ConnectSerial,
    Disconnect,
    Start,
    Stop,
    StartFieldCalibration,
    Command,
)
from nortek_nucleus_dvl.msg import (
    AHRS,
    INS,
    Altimeter,
    BottomTrack,
    CurrentProfile,
    FieldCalibration,
    IMU,
    Magnetometer,
)

from nucleus_driver import NucleusDriver


class NucleusNode():

    def __init__(self):
        rospy.init_node('nucleus_dvl', anonymous=True)

        self.nucleus_driver = NucleusDriver()

        self.connect_tcp_service = rospy.Service("nucleus_dvl/connect_tcp", Empty, self.connect_tcp_callback)
        self.connect_serial_service = rospy.Service("nucleus_dvl/connect_serial", ConnectSerial, self.connect_serial_callback)
        self.disconnect_service = rospy.Service("nucleus_dvl/disconnect", Empty, self.disconnect_callback)
        self.start_service = rospy.Service("nucleus_dvl/start", Empty, self.start_callback)
        self.start_field_calibration_service = rospy.Service("nucleus_dvl/field_calibration", Empty, self.start_field_calibration_callback)
        self.stop_service = rospy.Service("nucleus_dvl/stop", Empty, self.stop_callback)
        self.command_service = rospy.Service("nucleus_dvl/command", Command, self.command_callback)

        self.ahrs_publisher = rospy.Publisher("nucleus_dvl/ahrs", AHRS, queue_size=100)
        self.altimeter_publisher = rospy.Publisher("nucleus_dvl/altimeter", Altimeter, queue_size=100)
        self.bottom_track_publisher = rospy.Publisher("nucleus_dvl/bottom_track", BottomTrack, queue_size=100)
        self.water_track_publisher = rospy.Publisher("nucleus_dvl/water_track", BottomTrack, queue_size=100)
        self.current_profile_publisher = rospy.Publisher("nucleus_dvl/current_profile", CurrentProfile, queue_size=100)
        self.field_calibration_publisher = rospy.Publisher("nucleus_dvl/field_calibration", FieldCalibration, queue_size=100)
        self.imu_publisher = rospy.Publisher("nucleus_dvl/imu", IMU, queue_size=100)
        self.ins_publisher = rospy.Publisher("nucleus_dvl/ins", INS, queue_size=100)
        self.mag_publisher = rospy.Publisher("nucleus_dvl/magnetometer", Magnetometer, queue_size=100)
        
        self.packet_timer = rospy.Timer(rospy.Duration(0.001), self.packet_callback)

        rospy.loginfo("Nucleus Node initiated")

    def connect_tcp_callback(self, request):
        response = False
        self.nucleus_driver.set_tcp_configuration('192.168.2.150')
        status = self.nucleus_driver.connect(connection_type="tcp")

        if status:
            rospy.loginfo(f"Connected through TCP with host:192.168.2.150")
        else:
            rospy.loginfo(f"Failed to connect with host:192.168.2.150")

        response=status
        return []

    def connect_serial_callback(self, request):
        response = ConnectSerialResponse()
        self.nucleus_driver.set_serial_configuration(port=request.serial_port)
        status = self.nucleus_driver.connect(connection_type="serial")

        if status:
            rospy.loginfo(f"Connected through serial with serial port: {request.serial_port}")
        else:
            rospy.loginfo(f"Failed to connect through serial port: {request.serial_port}")

        response.status = status
        return response

    def disconnect_callback(self, request):
        response = False
        status = self.nucleus_driver.disconnect()

        if status:
            rospy.loginfo(f"Successfully disconnected from Nucleus")
        else:
            rospy.loginfo(f"Failed to disconnect from Nucleus")

        return []

    def start_callback(self, request):
        response=False
        if not self.nucleus_driver.connection.get_connection_status():
            rospy.loginfo(f"Nucleus is not connected")
            #reply = f"Nucleus is not connected"
        else:
            self.nucleus_driver.start_measurement()
            response=True
        if(response):
            rospy.loginfo("started")
        else: 
            rospy.logwarn("failed to decode response from start command")
        return []


    def start_field_calibration_callback(self, request):
        response = False

        if not self.nucleus_driver.connection.get_connection_status():
            rospy.loginfo(f"Nucleus is not connected")
        else:
            reply=self.nucleus_driver.start_fieldcal()
            response=True
            try:
                reply = reply[0].decode()
                rospy.loginfo(f"start field calibration reply")
            except Exception as e:
                rospy.logwarn("Failed to decode response from start field calibration command: {e}")

        return []

    def stop_callback(self, request):
        response = False

        if not self.nucleus_driver.connection.get_connection_status():
            rospy.loginfo(f"Nucleus is not connected")
            reply = f"Nucleus is not connected"
        else:
            reply = self.nucleus_driver.stop()
            response=True
        if (response):
            rospy.loginfo(f"stopped")
        else:
            rospy.logwarn(f"Failed to decode response from start command")

        return []

    def command_callback(self, request):
        response = CommandResponse()

        if not self.nucleus_driver.connection.get_connection_status():
            rospy.loginfo(f"Nucleus is not connected")
            response.reply = f"Nucleus is not connected"
        else:
            command = request.command
            reply = self.nucleus_driver.send_command(command=command)

            try:
                for entry in reply:
                    response.reply += entry.decode()

                rospy.loginfo(f"command reply: {response.reply}")
            except Exception as e:
                response.reply = f"Failed to decode response from command: {e}"

        return response

    def packet_callback(self, event):
        packet = self.nucleus_driver.read_packet()

        if packet is None:
            return

        if packet["id"] == 0xD2:

            ahrs_packet = AHRS()

            ahrs_packet.posix_time = packet["flags.posixTime"]
            ahrs_packet.timestamp = packet["timeStamp"]
            ahrs_packet.microseconds = packet["microSeconds"]

            ahrs_packet.serial_number = packet["serialNumber"]
            ahrs_packet.operation_mode = packet["operationMode"]

            ahrs_packet.fom_ahrs = packet["fomAhrs"]
            ahrs_packet.fom_fc1 = packet["fomFc1"]

            ahrs_packet.roll = packet["ahrsData.roll"]
            ahrs_packet.pitch = packet["ahrsData.pitch"]
            ahrs_packet.heading = packet["ahrsData.heading"]

            ahrs_packet.quaternion_w = packet["ahrsData.quaternionW"]
            ahrs_packet.quaternion_x = packet["ahrsData.quaternionX"]
            ahrs_packet.quaternion_y = packet["ahrsData.quaternionY"]
            ahrs_packet.quaternion_z = packet["ahrsData.quaternionZ"]

            ahrs_packet.rotation_matrix_0 = packet["ahrsData.rotationMatrix_0"]
            ahrs_packet.rotation_matrix_1 = packet["ahrsData.rotationMatrix_1"]
            ahrs_packet.rotation_matrix_2 = packet["ahrsData.rotationMatrix_2"]
            ahrs_packet.rotation_matrix_3 = packet["ahrsData.rotationMatrix_3"]
            ahrs_packet.rotation_matrix_4 = packet["ahrsData.rotationMatrix_4"]
            ahrs_packet.rotation_matrix_5 = packet["ahrsData.rotationMatrix_5"]
            ahrs_packet.rotation_matrix_6 = packet["ahrsData.rotationMatrix_6"]
            ahrs_packet.rotation_matrix_7 = packet["ahrsData.rotationMatrix_7"]
            ahrs_packet.rotation_matrix_0 = packet["ahrsData.rotationMatrix_0"]

            ahrs_packet.declination = packet["declination"]
            ahrs_packet.depth = packet["depth"]

            self.ahrs_publisher.publish(ahrs_packet)

        if packet["id"] == 0xDC:

            ins_packet = INS()

            ins_packet.posix_time = packet["flags.posixTime"]
            ins_packet.timestamp = packet["timeStamp"]
            ins_packet.microseconds = packet["microSeconds"]

            ins_packet.serial_number = packet["serialNumber"]
            ins_packet.operation_mode = packet["operationMode"]

            ins_packet.fom_ahrs = packet["fomAhrs"]
            ins_packet.fom_fc1 = packet["fomFc1"]

            ins_packet.roll = packet["ahrsData.roll"]
            ins_packet.pitch = packet["ahrsData.pitch"]
            ins_packet.heading = packet["ahrsData.heading"]

            ins_packet.quaternion_w = packet["ahrsData.quaternionW"]
            ins_packet.quaternion_x = packet["ahrsData.quaternionX"]
            ins_packet.quaternion_y = packet["ahrsData.quaternionY"]
            ins_packet.quaternion_z = packet["ahrsData.quaternionZ"]

            ins_packet.rotation_matrix_0 = packet["ahrsData.rotationMatrix_0"]
            ins_packet.rotation_matrix_1 = packet["ahrsData.rotationMatrix_1"]
            ins_packet.rotation_matrix_2 = packet["ahrsData.rotationMatrix_2"]
            ins_packet.rotation_matrix_3 = packet["ahrsData.rotationMatrix_3"]
            ins_packet.rotation_matrix_4 = packet["ahrsData.rotationMatrix_4"]
            ins_packet.rotation_matrix_5 = packet["ahrsData.rotationMatrix_5"]
            ins_packet.rotation_matrix_6 = packet["ahrsData.rotationMatrix_6"]
            ins_packet.rotation_matrix_7 = packet["ahrsData.rotationMatrix_7"]
            ins_packet.rotation_matrix_0 = packet["ahrsData.rotationMatrix_0"]

            ins_packet.declination = packet["declination"]
            ins_packet.depth = packet["depth"]

            ins_packet.fom_ins = packet["fomIns"]
            ins_packet.lat_long_is_valid = packet["statusIns.latLonIsValid"]
            ins_packet.course_over_ground = packet["courseOverGround"]
            ins_packet.temperature = packet["temperature"]
            ins_packet.pressure = packet["pressure"]
            ins_packet.altitude = packet["altitude"]
            ins_packet.latitude = packet["latitude"]
            ins_packet.longitude = packet["longitude"]
            ins_packet.position_frame_x = packet["positionFrameX"]
            ins_packet.position_frame_y = packet["positionFrameY"]
            ins_packet.position_frame_z = packet["positionFrameZ"]
            ins_packet.velocity_ned_x = packet["velocityNedX"]
            ins_packet.velocity_ned_y = packet["velocityNedY"]
            ins_packet.velocity_ned_z = packet["velocityNedZ"]
            ins_packet.velocity_nucleus_x = packet["velocityNucleusX"]
            ins_packet.velocity_nucleus_y = packet["velocityNucleusY"]
            ins_packet.velocity_nucleus_z = packet["velocityNucleusZ"]
            ins_packet.speed_over_ground = packet["speedOverGround"]
            ins_packet.turn_rate_x = packet["turnRateX"]
            ins_packet.turn_rate_y = packet["turnRateY"]
            ins_packet.turn_rate_z = packet["turnRateZ"]

            self.ins_publisher.publish(ins_packet)


        if packet["id"] == 0x87:

            mag_packet = Magnetometer()

            mag_packet.posix_time = packet["flags.posixTime"]
            mag_packet.timestamp = packet["timeStamp"]
            mag_packet.microseconds = packet["microSeconds"]

            mag_packet.is_compensated_for_hard_iron = packet["status.isCompensatedForHardIron"]
            mag_packet.dvl_active = packet["status.dvlActive"]
            mag_packet.dvl_acoustics_active = packet["status.dvlAcousticsActive"]
            mag_packet.dvl_transmitter_active = packet["status.dvlTransmitterActive"]

            mag_packet.magnetometer_x = packet["magnetometer.x"]
            mag_packet.magnetometer_y = packet["magnetometer.y"]
            mag_packet.magnetometer_z = packet["magnetometer.z"]

            self.mag_publisher.publish(mag_packet)

        if packet["id"] in [0xB4, 0xBE]:

            bottom_track_packet = BottomTrack()

            bottom_track_packet.posix_time = packet["flags.posixTime"]
            bottom_track_packet.timestamp = packet["timeStamp"]
            bottom_track_packet.microseconds = packet["microSeconds"]

            bottom_track_packet.beam_1_velocity_valid = packet["status.beam1VelocityValid"]
            bottom_track_packet.beam_2_velocity_valid = packet["status.beam2VelocityValid"]
            bottom_track_packet.beam_3_velocity_valid = packet["status.beam3VelocityValid"]
            bottom_track_packet.beam_1_distance_valid = packet["status.beam1DistanceValid"]
            bottom_track_packet.beam_2_distance_valid = packet["status.beam2DistanceValid"]
            bottom_track_packet.beam_3_distance_valid = packet["status.beam3DistanceValid"]
            bottom_track_packet.beam_1_fom_valid = packet["status.beam1FomValid"]
            bottom_track_packet.beam_2_fom_valid = packet["status.beam2FomValid"]
            bottom_track_packet.beam_3_fom_valid = packet["status.beam3FomValid"]
            bottom_track_packet.x_velocity_valid = packet["status.xVelocityValid"]
            bottom_track_packet.y_velocity_valid = packet["status.yVelocityValid"]
            bottom_track_packet.z_velocity_valid = packet["status.zVelocityValid"]
            bottom_track_packet.x_fom_valid = packet["status.xFomValid"]
            bottom_track_packet.y_fom_valid = packet["status.yFomValid"]
            bottom_track_packet.z_fom_valid = packet["status.zFomValid"]

            bottom_track_packet.serial_number = packet["serialNumber"]
            bottom_track_packet.sound_speed = packet["soundSpeed"]
            bottom_track_packet.temperature = packet["temperature"]
            bottom_track_packet.pressure = packet["pressure"]
            bottom_track_packet.velocity_beam_1 = packet["velocityBeam1"]
            bottom_track_packet.velocity_beam_2 = packet["velocityBeam2"]
            bottom_track_packet.velocity_beam_3 = packet["velocityBeam3"]
            bottom_track_packet.distance_beam_1 = packet["distanceBeam1"]
            bottom_track_packet.distance_beam_2 = packet["distanceBeam2"]
            bottom_track_packet.distance_beam_3 = packet["distanceBeam3"]
            bottom_track_packet.fom_beam_1 = packet["fomBeam1"]
            bottom_track_packet.fom_beam_2 = packet["fomBeam2"]
            bottom_track_packet.fom_beam_3 = packet["fomBeam3"]
            bottom_track_packet.dt_beam_1 = packet["dtBeam1"]
            bottom_track_packet.dt_beam_2 = packet["dtBeam2"]
            bottom_track_packet.dt_beam_3 = packet["dtBeam3"]
            bottom_track_packet.time_vel_beam_1 = packet["timeVelBeam1"]
            bottom_track_packet.time_vel_beam_2 = packet["timeVelBeam2"]
            bottom_track_packet.time_vel_beam_3 = packet["timeVelBeam3"]
            bottom_track_packet.velocity_x = packet["velocityX"]
            bottom_track_packet.velocity_y = packet["velocityY"]
            bottom_track_packet.velocity_z = packet["velocityZ"]
            bottom_track_packet.fom_x = packet["fomX"]
            bottom_track_packet.fom_y = packet["fomY"]
            bottom_track_packet.fom_z = packet["fomX"]
            bottom_track_packet.dt_xyz = packet["dtXYZ"]
            bottom_track_packet.time_vel_xyz = packet["timeVelXYZ"]

            if packet["id"] == 0xB4:
                self.bottom_track_publisher.publish(bottom_track_packet)

            elif packet["id"] == 0xBE:
                self.water_track_publisher.publish(bottom_track_packet)

        if packet["id"] == 0xAA:

            altimeter_packet = Altimeter()

            altimeter_packet.posix_time = packet["flags.posixTime"]
            altimeter_packet.timestamp = packet["timeStamp"]
            altimeter_packet.microseconds = packet["microSeconds"]

            altimeter_packet.altimeter_distance_valid = packet["status.altimeterDistanceValid"]
            altimeter_packet.altimeter_quality_valid = packet["status.altimeterQualityValid"]
            altimeter_packet.pressure_valid = packet["status.pressureValid"]
            altimeter_packet.temperature_valid = packet["status.temperatureValid"]

            altimeter_packet.serial_number = packet["serialNumber"]
            altimeter_packet.sound_speed = packet["soundSpeed"]
            altimeter_packet.temperature = packet["temperature"]
            altimeter_packet.pressure = packet["pressure"]
            altimeter_packet.altimeter_distance = packet["altimeterDistance"]
            altimeter_packet.altimeter_quality = packet["altimeterQuality"]

            self.altimeter_publisher.publish(altimeter_packet)

        if packet["id"] == 0xC0:

            current_profile_packet = CurrentProfile()

            current_profile_packet.posix_time = packet["flags.posixTime"]
            current_profile_packet.timestamp = packet["timeStamp"]
            current_profile_packet.microseconds = packet["microSeconds"]

            current_profile_packet.serial_number = packet["serialNumber"]
            current_profile_packet.sound_velocity = packet["soundVelocity"]
            current_profile_packet.temperature = packet["temperature"]
            current_profile_packet.pressure = packet["pressure"]
            current_profile_packet.cell_size = packet["cellSize"]
            current_profile_packet.blanking = packet["blanking"]
            current_profile_packet.number_of_cells = packet["numberOfCells"]
            current_profile_packet.ambiguity_velocity = packet["ambiguityVelocity"]

            velocity_data = list()
            amplitude_data = list()
            correlation_data = list()

            for key in packet.keys():
                if "velocityData" in key:
                    velocity_data.append(packet[key])

                elif "amplitudeData" in key:
                    amplitude_data.append(packet[key])

                elif "correlationData" in key:
                    correlation_data.append(packet[key])

            current_profile_packet.velocity_data = velocity_data
            current_profile_packet.amplitude_data = amplitude_data
            current_profile_packet.correlation_data = correlation_data

            self.current_profile_publisher.publish(current_profile_packet)

        if packet["id"] == 0x8B:

            field_calibration_packet = FieldCalibration()

            field_calibration_packet.posix_time = packet["flags.posixTime"]
            field_calibration_packet.timestamp = packet["timeStamp"]
            field_calibration_packet.microseconds = packet["microSeconds"]

            field_calibration_packet.points_used_in_estimation = packet["status.pointsUsedInEstimation"]

            field_calibration_packet.hard_iron_x = packet["hardIron.x"]
            field_calibration_packet.hard_iron_y = packet["hardIron.y"]
            field_calibration_packet.hard_iron_z = packet["hardIron.z"]
            field_calibration_packet.s_axis_0 = packet["sAxis_0"]
            field_calibration_packet.s_axis_1 = packet["sAxis_1"]
            field_calibration_packet.s_axis_2 = packet["sAxis_2"]
            field_calibration_packet.s_axis_3 = packet["sAxis_3"]
            field_calibration_packet.s_axis_4 = packet["sAxis_4"]
            field_calibration_packet.s_axis_5 = packet["sAxis_5"]
            field_calibration_packet.s_axis_6 = packet["sAxis_6"]
            field_calibration_packet.s_axis_7 = packet["sAxis_7"]
            field_calibration_packet.s_axis_8 = packet["sAxis_8"]
            field_calibration_packet.new_point_x = packet["newPoint.x"]
            field_calibration_packet.new_point_y = packet["newPoint.y"]
            field_calibration_packet.new_point_z = packet["newPoint.z"]
            field_calibration_packet.fom_field_calibration = packet["fomFieldCalibration"]
            field_calibration_packet.coverage = packet["coverage"]

            self.field_calibration_publisher.publish(field_calibration_packet)
        if packet["id"] == 0x82:
            imu_packet = IMU()
            imu_packet.posix_time = packet["flags.posixTime"]
            imu_packet.timestamp = packet["timeStamp"]
            imu_packet.microseconds = packet["microSeconds"]

            
            imu_packet.gyro_x = packet["gyro.x"]
            imu_packet.gyro_y = packet["gyro.y"]
            imu_packet.gyro_z = packet["gyro.z"]

            imu_packet.accelerometer_x = packet["accelerometer.x"]
            imu_packet.accelerometer_y = packet["accelerometer.y"]
            imu_packet.accelerometer_z = packet["accelerometer.z"]
            
            imu_packet.temperature = packet["temperature"]


            self.imu_publisher.publish(imu_packet)

    def connection_lost(self, exc):
        print('port closed')
        self.ins_publisher.unregister()
        self.altimeter_publisher.unregister()
        self.bottom_track_publisher.unregister()
        self.water_track_publisher.unregister()
        self.current_profile_publisher.unregister()
        self.field_calibration_publisher.unregister()
        self.imu_publisher.unregister()


if __name__ == '__main__':
    nucleus_node = NucleusNode()
    rospy.spin()

