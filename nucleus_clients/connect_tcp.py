import sys
import rospy
from nortek_nucleus_dvl.srv import ConnectTcp

class ClientConnectTcp:

    def __init__(self):
        rospy.init_node('connect_tcp_client')
        self.client = rospy.ServiceProxy('nucleus_node/connect_tcp', ConnectTcp)
        rospy.wait_for_service('nucleus_node/connect_tcp')

    def send_request(self, host, password=None):
        try:
            response = self.client(host, password)
            return response.status
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

def main():
    try:
        host = str(sys.argv[1])
    except IndexError:
        print('First argument "host" must be specified')
        return
    except Exception as e:
        print(f'Invalid argument for host: {e}')
        return

    try:
        password = str(sys.argv[2])
    except IndexError:
        print('Second argument "password" must be specified')
        return
    except Exception as e:
        print(f'Invalid argument for password: {e}')
        return
    
    client = ClientConnectTcp()
    status = client.send_request(host, password)

    rospy.loginfo(f'Successfully made the connect tcp call with status: {status}')

if __name__ == '__main__':
    main()

