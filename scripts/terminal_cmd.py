#!/usr/bin/python
import subprocess

def send_terminal_command(command):
    try:
        # Run the terminal command
        result = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        # Print the command output
        print("Command output:", result.stdout)
    except subprocess.CalledProcessError as e:
        # Handle any errors
        print("Error:", e)
        print("Command stderr:", e.stderr)

def delete_robot():
    delete_robot = "ign service -s /world/demo_world/remove --reqtype ignition.msgs.Entity --reptype ignition.msgs.Boolean --timeout 300 --req 'name: \"bouncy\" type: MODEL'"
    restart_world = "ign service -s /world/demo_world/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 2000 --req 'reset: {all: true}'"
    send_terminal_command(delete_robot)

