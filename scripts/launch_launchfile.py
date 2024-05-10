#!/usr/bin/python
import launch
import sys
import threading
from launchfile import generate_launch_description
import time

class launchfile():
    def __init__(self):
        self.launch_description = generate_launch_description()
        self.launch_bouncy = launch.launch_service.LaunchService(argv=[*sys.argv[1:]])
        self.launch_bouncy.include_launch_description(self.launch_description)

    def run(self):
        self.launch_bouncy.run()

    def shutdown(self):
        self.launch_bouncy.shutdown()

def waitnstop(bouncy_bot):
    print("start")
    time.sleep(10)
    print("done")
    bouncy_bot.shutdown()

def launch_robot(side_function):

    launch_bouncy = launchfile()

    launch_thread = threading.Thread(target=side_function, args=(launch_bouncy,))
    launch_thread.start()

    launch_bouncy.run()

    launch_thread.join()

    launch_bouncy.shutdown()
