import rospy
from nortek_nucleus_dvl.msg import AHRS


class SubscriberAhrsPackets:
    def __init__(self, callback_function, queue_size=10):
        self.node_name = 'ahrs_packets_subscriber'
        rospy.init_node(self.node_name, anonymous=True)
        self.subscription = rospy.Subscriber(
            'nucleus_node/ahrs_packets',
            AHRS,
            callback_function,
            queue_size=queue_size
        )

    def subscribe(self):
        rospy.spin()

def main():
    def ahrs_packet_callback(ahrs):
        try:
            DIGIT_LENGTH = 5
            formatted_ahrs = []

            for data in [ahrs.roll, ahrs.pitch, ahrs.heading]:
                integral_length = len(str(data).split('.')[0])
                decimal_length = max(0, DIGIT_LENGTH - integral_length)

                formatted_ahrs.append(f'{data:.{decimal_length}f}')

            rospy.loginfo(f'roll: {formatted_ahrs[0]} | pitch: {formatted_ahrs[1]} | heading: {formatted_ahrs[2]}')
        
        except Exception as e:
            rospy.loginfo(f'roll: {round(ahrs.roll, 3)} | pitch: {round(ahrs.pitch, 3)} | heading: {round(ahrs.heading, 3)}')
            rospy.logerr(f'Error processing AHRS data: {e}')

    subscriber = SubscriberAhrsPackets(callback_function=ahrs_packet_callback)
    subscriber.subscribe()

if __name__ == '__main__':
    main()

