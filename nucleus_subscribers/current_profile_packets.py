import rospy
from nortek_nucleus_dvl.msg import CurrentProfile

class SubscriberCurrentProfilePackets:
    def __init__(self, callback_function, queue_size=10):
        self.node_name = 'current_profile_packets_subscriber'
        rospy.init_node(self.node_name, anonymous=True)
        self.subscription = rospy.Subscriber(
            'nucleus_node/current_profile_packets',
            CurrentProfile,
            callback_function,
            queue_size=queue_size
        )

    def subscribe(self):
        rospy.spin()

def main():
    def current_profile_packet_callback(current_profile):
        try:
            rospy.loginfo(f'velocity_data: {current_profile.velocity_data}')
        except Exception as e:
            rospy.logerr(f'Error processing CurrentProfile data: {e}')

    subscriber = SubscriberCurrentProfilePackets(callback_function=current_profile_packet_callback)
    subscriber.subscribe()

if __name__ == '__main__':
    main()

