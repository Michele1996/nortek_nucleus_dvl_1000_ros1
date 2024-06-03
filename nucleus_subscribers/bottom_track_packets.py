import rospy
from nortek_nucleus_dvl.msg import BottomTrack

class SubscriberBottomTrackPackets:
    def __init__(self, callback_function, queue_size=10):
        self.node_name = 'bottom_track_packets_subscriber'
        rospy.init_node(self.node_name, anonymous=True)
        self.subscription = rospy.Subscriber(
            'nucleus_node/bottom_track_packets',
            BottomTrack,
            callback_function,
            queue_size=queue_size
        )

    def subscribe(self):
        rospy.spin()

def main():
    def bottom_track_packet_callback(bottom_track):
        try:
            DIGIT_LENGTH = 5
            formatted_bottom_track = []

            for data in [bottom_track.velocity_x, bottom_track.velocity_y, bottom_track.velocity_z, bottom_track.fom_x, bottom_track.fom_y, bottom_track.fom_z]:
                integral_length = len(str(data).split('.')[0])
                decimal_length = max(0, DIGIT_LENGTH - integral_length)

                formatted_bottom_track.append(f'{data:.{decimal_length}f}')

            rospy.loginfo(f'velocity_x: {formatted_bottom_track[0]} | velocity_y: {formatted_bottom_track[1]} | velocity_z: {formatted_bottom_track[2]} | fom_x: {formatted_bottom_track[3]} | fom_y: {formatted_bottom_track[4]} | fom_z: {formatted_bottom_track[5]}')
        
        except Exception as e:
            rospy.loginfo(f'velocity_x: {round(bottom_track.velocity_x, 3)} | velocity_y: {round(bottom_track.velocity_y, 3)} | velocity_z: {round(bottom_track.velocity_z, 3)} | fom_x: {round(bottom_track.fom_x, 3)} | fom_y: {round(bottom_track.fom_y, 3)} | fom_z: {round(bottom_track.fom_z, 3)}')
            rospy.logerr(f'Error processing BottomTrack data: {e}')

    subscriber = SubscriberBottomTrackPackets(callback_function=bottom_track_packet_callback)
    subscriber.subscribe()

if __name__ == '__main__':
    main()

