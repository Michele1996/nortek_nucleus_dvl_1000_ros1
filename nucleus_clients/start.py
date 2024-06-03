import sys
import rospy
from nortek_nucleus_dvlinterfaces.srv import Start

class ClientStart:

    def __init__(self):
        rospy.init_node('start_client')
        self.client = rospy.ServiceProxy('nucleus_node/start', Start)
        rospy.wait_for_service('nucleus_node/start')
        self.request = Start.Request()

    def send_request(self):
        try:
            response = self.client(self.request)
            return response.reply
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

def main():
    client = ClientStart()
    reply = client.send_request()

    rospy.loginfo(f'Successfully made the start call with status: {reply}')

if __name__ == '__main__':
    main()

