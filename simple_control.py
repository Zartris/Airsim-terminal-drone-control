import airsim
import pprint
import cv2
import numpy as np
from pynput.keyboard import Key, Listener
from KeyController import KeyController

# Commands:
ARM = "arm"
DISARM = "disarm"
MOVE = "move"
MOVE_PATH = "moveonpath"
HOME = "home"
STATE = "state"
TAKEOFF = "takeoff"
STOP = "stop"
KEYBOARD_CONTROL = "kc"
FORWARD_FORCE = 1
BACKWARD_FORCE = -1
RIGHT_FORCE = 1
LEFT_FORCE = -1


class SimpleTerminalController:
    def __init__(self, verbatim=True, maxmin_velocity=4, drive_type=airsim.DrivetrainType.ForwardOnly):
        # Should this class print to terminal
        self.verbatim = verbatim
        self.DriveType = drive_type
        # connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.confirm_connection()
        # Segmentation setup
        self.setup_segmentation_colors()

        # Movement and constraints:
        self.vx = 0
        self.vy = 0
        self.vz = 0

        self.maxmin_vel = maxmin_velocity

    def confirm_connection(self):
        self.client.confirmConnection()
        self.client.enableApiControl(True)

    def setup_segmentation_colors(self):
        # Finding regexp GameObject name and set the ID
        success = self.client.simSetSegmentationObjectID("SM_Floor20m[\w]*", 100, True)
        print("Change of color =", success)

    def takeoff(self):
        state = self.client.getMultirotorState()
        print("Takeoff received")
        if state.landed_state == airsim.LandedState.Landed:
            print("taking off...")
            self.client.takeoffAsync().join()
        else:
            self.client.hoverAsync().join()

    def arm(self):
        print("Arm received")
        self.client.armDisarm(True)

    def disarm(self):
        print("Disarm received")
        self.client.armDisarm(False)

    def move_to_position(self, args: list):
        print("Move received")
        if len(args) != 5:
            print("Move needs 5 args")
            return
        self.client.enableApiControl(True)
        print("Move args:", float(args[1]), float(args[2]), float(args[3]), float(args[4]))
        self.client.moveToPositionAsync(x=float(args[1]), y=float(args[2]), z=float(args[3]),
                                        velocity=float(args[4]), drivetrain=self.DriveType).join()
        self.client.hoverAsync().join()
        print("Moved!")

    def move_on_path(self, args: list):
        print("MoveOnPath received")
        if len(args) % 3 != 2:
            print("Move needs 3 args per position args")
            return
        # Have to make sure it is enabled:
        self.client.enableApiControl(True)
        iterations = (len(args) - 2) / 3
        path = []
        for i in range(int(iterations)):
            point = airsim.Vector3r(float(args[(i * 3) + 1]),
                                    float(args[(i * 3) + 2]),
                                    float(args[(i * 3) + 3]))
            path.append(point)
            if self.verbatim:
                print("path point added", str(point))
        try:
            result = self.client.moveOnPathAsync(path, float(args[-1]), 120,
                                                 airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0),
                                                 20,
                                                 1).join()
        except:
            errorType, value, traceback = airsim.sys.exc_info()
            print("moveOnPath threw exception: " + str(value))
            pass
        self.client.hoverAsync().join()
        print("Path moved!")

    def home(self):
        print("Home received")
        self.client.goHomeAsync()
        self.client.armDisarm(False)

    def stop(self):
        self.client.goHomeAsync()
        self.client.armDisarm(False)
        self.client.reset()

    def handle_key_pressed(self, keysToCheck: list, pressedKeys: list, current_vel: float) -> float:
        new_vel = current_vel
        positive_axis_press = keysToCheck[0] in pressedKeys
        negative_axis_press = keysToCheck[1] in pressedKeys

        if positive_axis_press and negative_axis_press:
            return new_vel

        if positive_axis_press:
            return round(number=float(np.clip(new_vel + 0.25, - self.maxmin_vel, self.maxmin_vel)), ndigits=2)

        if negative_axis_press:
            return round(number=float(np.clip(new_vel - 0.25, - self.maxmin_vel, self.maxmin_vel)), ndigits=2)

        # nothing is pressed, smoothly lowering the value
        return round(number=float(np.clip(new_vel * 0.75, - self.maxmin_vel, self.maxmin_vel)), ndigits=2)

    def enter_keyboard_control(self):
        print("You entered the keyboard mode. Press 'b' to return.")
        kc = KeyController()
        self.client.enableApiControl(True)
        while kc.thread.isAlive():
            self.client.cancelLastTask()
            self.client.enableApiControl(True)
            keys = kc.get_key_pressed()
            # Keys explained: keys[0] is the positive axis (forward, right, down)
            # keys[1] is the negative axis (backwards, left, up)
            quad_vel = self.client.getMultirotorState().kinematics_estimated.linear_velocity
            self.vx = self.handle_key_pressed(keysToCheck=['w', 's'], pressedKeys=keys, current_vel=quad_vel.x_val)
            self.vy = self.handle_key_pressed(keysToCheck=['d', 'a'], pressedKeys=keys, current_vel=quad_vel.y_val)
            self.vz = self.handle_key_pressed(keysToCheck=['e', 'q'], pressedKeys=keys, current_vel=quad_vel.z_val)
            print("current: \n vx:{0}, nvx:{1}\n vy:{2}, nvy:{3}\n vz:{4}, nvz:{5}\n".format(quad_vel.x_val, self.vx,
                                                                                             quad_vel.y_val, self.vy,
                                                                                             quad_vel.z_val, self.vz))
            self.client.moveByVelocityAsync(self.vx, self.vy, self.vz, 0.1, airsim.DrivetrainType.ForwardOnly,
                                            airsim.YawMode(False, 0)).join()
            # airsim.time.sleep(0.2)
        print("'b' has been pressed and the console control is back")
        self.client.hoverAsync().join()

    def print_stats(self):
        state = self.client.getMultirotorState()
        s = pprint.pformat(state)
        print("state: %s" % s)

        imu_data = self.client.getImuData()
        s = pprint.pformat(imu_data)
        print("imu_data: %s" % s)

        barometer_data = self.client.getBarometerData()
        s = pprint.pformat(barometer_data)
        print("barometer_data: %s" % s)

        magnetometer_data = self.client.getMagnetometerData()
        s = pprint.pformat(magnetometer_data)
        print("magnetometer_data: %s" % s)

        gps_data = self.client.getGpsData()
        s = pprint.pformat(gps_data)
        print("gps_data: %s" % s)

    def run(self):
        while True:
            command = input()
            args = command.split(" ")
            print("Args given", args)
            command_type = args[0]
            if command_type.lower() == ARM:
                self.arm()
            elif command_type.lower() == DISARM:
                self.disarm()
            elif command_type.lower() == MOVE:
                self.move_to_position(args)
            elif command_type.lower() == MOVE_PATH:
                self.move_on_path(args)
            elif command_type.lower() == HOME:
                self.home()
            elif command_type.lower() == TAKEOFF:
                self.takeoff()
            elif command_type.lower() == STATE:
                self.print_stats()
            elif command_type.lower() == KEYBOARD_CONTROL:
                self.enter_keyboard_control()
            elif command_type.lower() == STOP:
                self.stop()
                break
            else:
                print("The command given is not a valid command.")

        # that's enough fun for now. let's quit cleanly
        airsim.wait_key("When ready to kill")
        self.client.enableApiControl(False)


if __name__ == '__main__':
    controller = SimpleTerminalController(maxmin_velocity=20)
    controller.run()
