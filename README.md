The intended folder is final_panorama_action (UNTESTED)

The Working Mechanism (Step-by-Step)
Phase 1: The Setup

    Start: You launch the panorama_action_server. It sits idle, subscribing to /camera/image_raw and /servo_angle.

    Buffer: It constantly updates a variable latest_image in its memory with the newest frame from the camera, 30 times a second. It effectively has one eye open at all times.

Phase 2: The Command

    Trigger: You send the Goal: total_angle_sweep: 180, angle_interval: 20.

    Calculation: The server calculates the "Waypoints": 0, 20, 40, 60, ..., 180.

Phase 3: The "Handshake" Loop (The Core Mechanism)

This is the loop that runs for each waypoint (e.g., let's look at the 20° target).

    The Wait (Standoff):
    The Action Server enters a "Check Mode." It looks at the /servo_angle topic.

        Server: "Is the angle 20 yet?"

        Topic: "No, it's 0."

        Server: "Okay, I'll wait." (It sleeps for 50ms and checks again).

        Feedback: It sends a message to your terminal: "Status: Waiting for servo to reach 20.0".

    The Movement (External):
    This is where you (or your navigation node) act.

        You manually rotate the rover (or drive the servo) to the right.

        Your hardware driver publishes the new angle to /servo_angle: 5... 10... 15... 19... 20.

    The Trigger:

        Server: "Is the angle 20 yet?"

        Topic: "Yes, it is 20.1."

        MATCH! The server sees the number matches the target (within a +/- 2 degree tolerance).

    The Capture:

        The Server immediately freezes the latest_image from memory.

        It saves it as img_1.jpg in the temp folder.

        It updates its internal counter to the next target: 40°.

Phase 4: Stitching & Finish

    Once the server captures the final image at 180°, it breaks the loop.

    It passes all the saved file paths to the OpenCV Stitcher.

    It saves the final result.jpg.

    It sends a Result message back to you: "Success. Saved to..."


Usage:

Build:
Bash

cd ~/ros2_ws
colcon build --packages-select final_panorama_action
source install/setup.bash

Run (Terminal 1):
Bash

ros2 run final_panorama_action panorama_action_server

If you want to enable the split logic: ... --ros-args -p divide_images:=true

Trigger Action (Terminal 2):
Bash

ros2 action send_goal /capture_panorama final_panorama_action/action/CapturePanorama "{total_angle_sweep: 180.0, angle_interval: 20.0}"

Simulate Rover Movement (Terminal 3):
Publish the angles manually as you rotate the robot.
Bash

ros2 topic pub --once /servo_angle std_msgs/msg/Float32 "{data: 0.0}"
# (Wait for capture...)
ros2 topic pub --once /servo_angle std_msgs/msg/Float32 "{data: 20.0}"
# (Repeat...)
