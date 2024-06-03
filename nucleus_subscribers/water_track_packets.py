import rospy
from nortek_nucleus_dvl.msg import BottomTrack

class SubscriberWaterTrackPackets:
    def __init__(self, callback_function, queue_size=10):
        self.node_name = 'water_track_packets_subscriber'
        rospy.init_node(self.node_name, anonymous=True)
        self.subscription = rospy.Subscriber(
            'nucleus_node/water_track_packets',
            BottomTrack,
            callback_function,
            queue_size=queue_size
        )

    def subscribe(self):
        rospy.spin()

def main():
    def water_track_packet_callback(water_track):
        try:
            DIGIT_LENGTH = 5
            formatted_water_track = []

            for data in [
                water_track.velocity_x,
                water_track.velocity_y,
                water_track.velocity_z,
                water_track.fom_x,
                water_track.fom_y,
                water_track.fom_z,
            ]:
                integral_length = len(str(data).split('.')[0])
                decimal_length = max(0, DIGIT_LENGTH - integral_length)

                formatted_water_track.append(f'{data:.{decimal_length}f}')

            rospy.loginfo(
                f'velocity_x: {formatted_water_track[0]} | velocity_y: {formatted_water_track[1]} | velocity_z: {formatted_water_track[2]} | fom_x: {formatted_water_track[3]} | fom_y: {formatted_water_track[4]} | fom_z: {formatted_water_track[5]}'
            )

        except Exception:
            rospy.loginfo(
                f'velocity_x: {round(water_track.velocity_x, 3)} | velocity_y: {round(water_track.velocity_y, 3)} | velocity_z: {round(water_track.velocity_z, 3)} | fom_x: {round(water_track.fom_x, 3)} | fom_y: {round(water_track.fom_y, 3)} | fom_z: {round(water_track.fom_z, 3)}'
            )

    subscriber = SubscriberWaterTrackPackets(callback_function=water_track_packet_callback)
    subscriber.subscribe()

if __name__ == '__main__':
    main()

