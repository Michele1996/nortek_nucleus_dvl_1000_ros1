import rospy
from nortek_nucleus_dvl.msg import Magnetometer

class SubscriberMagnetometerPackets:
    def __init__(self, callback_function, queue_size=10):
        self.node_name = 'magnetometer_packets_subscriber'
        rospy.init_node(self.node_name, anonymous=True)
        self.subscription = rospy.Subscriber(
            'nucleus_node/magnetometer_packets',
            Magnetometer,
            callback_function,
            queue_size=queue_size
        )

    def subscribe(self):
        rospy.spin()

def main():
    def mag_packet_callback(mag):
        try:
            DIGIT_LENGTH = 5
            formatted_mag = []

            for data in [mag.magnetometer_x, mag.magnetometer_y, mag.magnetometer_z]:
                integral_length = len(str(data).split('.')[0])
                decimal_length = max(0, DIGIT_LENGTH - integral_length)

                formatted_mag.append(f'{data:.{decimal_length}f}')

            rospy.loginfo(f'magnetometer_x: {formatted_mag[0]} | magnetometer_y: {formatted_mag[1]} | magnetometer_z: {formatted_mag[2]}')

        except Exception:
            rospy.loginfo(f'magnetometer_x: {round(mag.magnetometer_x, 3)} | magnetometer_y: {round(mag.magnetometer_y, 3)} | magnetometer_z: {round(mag.magnetometer_z, 3)}')

    subscriber = SubscriberMagnetometerPackets(callback_function=mag_packet_callback)
    subscriber.subscribe()

if __name__ == '__main__':
    main()

