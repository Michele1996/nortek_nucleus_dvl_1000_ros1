import rospy
from nortek_nucleus_dvl.msg import FieldCalibration

class SubscriberFieldCalibrationPackets:
    def __init__(self, callback_function, queue_size=10):
        self.node_name = 'field_calibration_packets_subscriber'
        rospy.init_node(self.node_name, anonymous=True)
        self.subscription = rospy.Subscriber(
            'nucleus_node/field_calibration_packets',
            FieldCalibration,
            callback_function,
            queue_size=queue_size
        )

    def subscribe(self):
        rospy.spin()

def main():
    def field_calibration_packet_callback(field_calibration):
        try:
            DIGIT_LENGTH = 5
            formatted_field_calibration = []

            for data in [field_calibration.hard_iron_x, field_calibration.hard_iron_y, field_calibration.hard_iron_z]:
                integral_length = len(str(data).split('.')[0])
                decimal_length = max(0, DIGIT_LENGTH - integral_length)

                formatted_field_calibration.append(f'{data:.{decimal_length}f}')

            rospy.loginfo(f'hard_iron_x: {formatted_field_calibration[0]} | hard_iron_y: {formatted_field_calibration[1]} | hard_iron_z: {formatted_field_calibration[2]}')

        except Exception:
            rospy.loginfo(f'hard_iron_x: {round(field_calibration.hard_iron_x, 3)} | hard_iron_y: {round(field_calibration.hard_iron_y, 3)} | hard_iron_z: {round(field_calibration.hard_iron_z, 3)}')

    subscriber = SubscriberFieldCalibrationPackets(callback_function=field_calibration_packet_callback)
    subscriber.subscribe()

if __name__ == '__main__':
    main()

