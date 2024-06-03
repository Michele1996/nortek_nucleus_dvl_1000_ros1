import sys
import rospy
from nortek_nucleus_dvl.srv import Command

class ClientCommand:

    def __init__(self):
        rospy.init_node('command_client')
        self.client = rospy.ServiceProxy('nucleus_node/command', Command)
        rospy.wait_for_service('nucleus_node/command')

    def send_request(self, command):
        try:
            response = self.client(command)
            return response.reply
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

def main():
    try:
        command = str(sys.argv[1])
    except IndexError:
        print('Argument "command" must be specified')
        return
    except Exception as e:
        print(f'Invalid argument: {e}')
        return

    client = ClientCommand()
    response = client.send_request(command)

    rospy.loginfo(response)

if __name__ == '__main__':
    main()

