import rospy
from nortek_nucleus_dvl.msg import Altimeter


class SubscriberAltimeterPackets:
    def __init__(self, callback_function, queue_size=10):
        self.node_name = 'altimeter_packets_subscriber'
        rospy.init_node(self.node_name, anonymous=True)
        self.subscription = rospy.Subscriber(
            'nucleus_node/altimeter_packets',
            Altimeter,
            callback_function,
            queue_size=queue_size
        )

    def subscribe(self):
        rospy.spin()

def main():
    def altimeter_packet_callback(altimeter):
        try:
            DIGIT_LENGTH = 5
            formatted_altimeter = []

            for data in [altimeter.altimeter_distance, altimeter.altimeter_quality, altimeter.temperature, altimeter.pressure]:
                integral_length = len(str(data).split('.')[0])
                decimal_length = max(0, DIGIT_LENGTH - integral_length)

                formatted_altimeter.append(f'{data:.{decimal_length}f}')

            rospy.loginfo(f'distance: {formatted_altimeter[0]} | quality: {formatted_altimeter[1]} | temperature: {formatted_altimeter[2]} | pressure: {formatted_altimeter[3]}')
        
        except Exception as e:
            rospy.loginfo(f'distance: {round(altimeter.altimeter_distance, 3)} | quality: {round(altimeter.altimeter_quality, 3)} | temperature: {round(altimeter.temperature, 3)} | pressure: {round(altimeter.pressure, 3)}')
            rospy.logerr(f'Error processing Altimeter data: {e}')

    subscriber = SubscriberAltimeterPackets(callback_function=altimeter_packet_callback)
    subscriber.subscribe()

if __name__ == '__main__':
    main()

