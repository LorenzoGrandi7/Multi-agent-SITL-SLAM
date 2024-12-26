"""
Copyright 2024 Lorenzo Grandi

GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007

Copyright (C) 2007 Free Software Foundation, Inc. <https://fsf.org/>
Everyone is permitted to copy and distribute verbatim copies
of this license document, but changing it is not allowed.
"""

#!/usr/bin/env python3

import subprocess
import time

def main():
    # List of commands to run
    commands = [
        # Run the Micro XRCE-DDS Agent
        "MicroXRCEAgent udp4 -p 8888",

        # Run the PX4 SITL simulation with spawning positions
        # "cd ~/PX4-Autopilot && make px4_sitl gazebo-classic_iris_depth_camera__our_empty"
        "cd ~/PX4-Autopilot/Tools/simulation/gazebo-classic && ./sitl_multiple_run.sh -s iris_depth_camera:1:0:0,iris_depth_camera:1:3:3 -w our_empty"
    ]

    # Loop through each command in the list
    for command in commands:
        # Each command is run in a new tab of the gnome-terminal
        subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
        
        # Pause between each command
        time.sleep(1)

if __name__ == "__main__":
    main()
