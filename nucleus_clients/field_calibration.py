import sys
import rospy
from nortek_nucleus_dvl.srv import StartFieldCalibration

class ClientFieldCalibration:

    def __init__(self):
        rospy.init_node('field_calibration_client')
        self.client = rospy.ServiceProxy('nucleus_node/field_calibration', StartFieldCalibration)
        rospy.wait_for_service('nucleus_node/field_calibration')
        self.request = StartFieldCalibration.Request()

    def send_request(self):
        try:
            response = self.client(self.request)
            return response.reply
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

def main():
    client = ClientFieldCalibration()
    reply = client.send_request()

    rospy.loginfo(f'Successfully made the field_calibration call with status: {reply}')

if __name__ == '__main__':
    main()

