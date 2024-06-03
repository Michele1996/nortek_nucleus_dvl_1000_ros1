import sys
import rospy
from rospy import ServiceException
from nortek_nucleus_dvl.srv import Stop

class ClientStop:

    def __init__(self):
        rospy.init_node('stop_client')
        self.client = rospy.ServiceProxy('nucleus_node/stop', Stop)
        rospy.wait_for_service('nucleus_node/stop')
        self.request = Stop.Request()

    def send_request(self):
        try:
            response = self.client(self.request)
            return response.reply
        except ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

def main():
    client = ClientStop()
    reply = client.send_request()

    rospy.loginfo(f'Successfully made the stop call with status: {reply}')

if __name__ == '__main__':
    main()

