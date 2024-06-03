import sys
import rospy
from nortek_nucleus_dvl.srv import ConnectSerial

class ClientConnectSerial:

    def __init__(self):
        rospy.init_node('connect_serial_client')
        self.client = rospy.ServiceProxy('nucleus_node/connect_serial', ConnectSerial)
        rospy.wait_for_service('nucleus_node/connect_serial')

    def send_request(self, serial_port):
        try:
            response = self.client(serial_port)
            return response.status
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

def main():
    try:
        serial_port = str(sys.argv[1])
    except IndexError:
        print('Argument "serial_port" must be specified')
        return
    except Exception as e:
        print(f'Invalid argument for serial_port: {e}')
        return
    
    client = ClientConnectSerial()
    status = client.send_request(serial_port)

    rospy.loginfo(status)

if __name__ == '__main__':
    main()

