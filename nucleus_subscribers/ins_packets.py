import rospy
from nortek_nucleus_dvl.msg import INS

class SubscriberInsPackets:
    def __init__(self, callback_function, queue_size=10):
        self.node_name = 'ins_packets_subscriber'
        rospy.init_node(self.node_name, anonymous=True)
        self.subscription = rospy.Subscriber(
            'nucleus_node/ins_packets',
            INS,
            callback_function,
            queue_size=queue_size
        )

    def subscribe(self):
        rospy.spin()

def main():
    def ins_packet_callback(ins):
        try:
            DIGIT_LENGTH = 5
            formatted_ins = []

            for data in [ins.position_frame_x, ins.position_frame_y, ins.position_frame_z, ins.altitude, ins.latitude, ins.longitude]:
                integral_length = len(str(data).split('.')[0])
                decimal_length = max(0, DIGIT_LENGTH - integral_length)

                formatted_ins.append(f'{data:.{decimal_length}f}')

            rospy.loginfo(f'position x: {formatted_ins[0]} | position y: {formatted_ins[1]} | position z: {formatted_ins[2]} | altitude: {formatted_ins[3]} | latitude: {formatted_ins[4]} | longitude: {formatted_ins[5]}')

        except Exception:
            rospy.loginfo(f'position x: {round(ins.position_frame_x, 3)} | position y: {round(ins.position_frame_y, 3)} | position z: {round(ins.position_frame_z, 3)} | altitude: {round(ins.altitude, 3)} | latitude: {round(ins.latitude, 3)} | longitude: {round(ins.longitude, 3)}')

    subscriber = SubscriberInsPackets(callback_function=ins_packet_callback)
    subscriber.subscribe()

if __name__ == '__main__':
    main()

