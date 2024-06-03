import sys
import rospy
from nortek_nucleus_dvl.srv import Disconnect

class ClientDisconnect:

    def __init__(self):
        rospy.init_node('disconnect_client')
        self.client = rospy.ServiceProxy('nucleus_node/disconnect', Disconnect)
        rospy.wait_for_service('nucleus_node/disconnect')
        self.request = Disconnect.Request()

    def send_request(self):
        try:
            response = self.client(self.request)
            return response.status
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

def main():
    client = ClientDisconnect()
    status = client.send_request()

    rospy.loginfo(f'Successfully made the disconnect serial call with status: {status}')

if __name__ == '__main__':
    main()

