#!/usr/bin/python
import launch
import sys
import threading
from launchfile import generate_launch_description
import time

class launchfile():
    def __init__(self, side_function, args):
        self.launch_description = generate_launch_description()
        self.launch_bouncy = launch.launch_service.LaunchService(argv=[*sys.argv[1:]])
        self.launch_bouncy.include_launch_description(self.launch_description)
        self.side_function = side_function
        self.args = args
        self.result=0.0

    def run(self):
        self.launch_bouncy.run()

    def shutdown(self):
        self.launch_bouncy.shutdown()

    def launch_robot(self):

        launch_thread = threading.Thread(target=self.run_side_function, args=self.args)
        launch_thread.start()

        self.run()

        launch_thread.join()

    def run_side_function(self, q_agent, interact_with_env, render=False):
        self.result=self.side_function(q_agent, interact_with_env, render)

        self.shutdown()

def wait(a,b,c):
        time.sleep(4)
        return (a,b,c)

if __name__ == "__main__":
    bouncy_bot = launchfile(wait,(1,2,3))
    print("\n\nstart\n\n")
    bouncy_bot.launch_robot()
    print("\n\nend\n\n")
    print(bouncy_bot.result)


