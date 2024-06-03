import rospy
from nortek_nucleus_dvl.msg import IMU

class SubscriberImuPackets:
    def __init__(self, callback_function, queue_size=10):
        self.node_name = 'imu_packets_subscriber'
        rospy.init_node(self.node_name, anonymous=True)
        self.subscription = rospy.Subscriber(
            'nucleus_node/imu_packets',
            IMU,
            callback_function,
            queue_size=queue_size
        )

    def subscribe(self):
        rospy.spin()

def main():
    def imu_packet_callback(imu):
        try:
            DIGIT_LENGTH = 5
            formatted_imu = []

            for data in [imu.accelerometer_x, imu.accelerometer_y, imu.accelerometer_z, imu.gyro_x, imu.gyro_y, imu.gyro_z]:
                integral_length = len(str(data).split('.')[0])
                decimal_length = max(0, DIGIT_LENGTH - integral_length)

                formatted_imu.append(f'{data:.{decimal_length}f}')

            rospy.loginfo(f'accelerometer_x: {formatted_imu[0]} | accelerometer_y: {formatted_imu[1]} | accelerometer_z: {formatted_imu[2]} | gyro_x: {formatted_imu[3]} | gyro_y: {formatted_imu[4]} | gyro_z: {formatted_imu[5]}')

        except Exception:
            rospy.loginfo(f'accelerometer_x: {round(imu.accelerometer_x, 3)} | accelerometer_y: {round(imu.accelerometer_y, 3)} | accelerometer_z: {round(imu.accelerometer_z, 3)} | gyro_x: {round(imu.gyro_x, 3)} | gyro_y: {round(imu.gyro_y, 3)} | gyro_z: {round(imu.gyro_z, 3)}')

    subscriber = SubscriberImuPackets(callback_function=imu_packet_callback)
    subscriber.subscribe()

if __name__ == '__main__':
    main()

