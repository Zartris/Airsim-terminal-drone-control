import pprint
import string

import airsim
import numpy as np
from pynput.keyboard import KeyCode

from KeyController import KeyController

# TIMEOUT
from airsim_functions.orbit import OrbitNavigator

TIMEOUT = 1200  # 20 miniuts

# Mesh ID's
BG = 0
LAND = 100
WATER = 200
SHIP = 300

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
ORBIT = "inspect"
FORWARD_FORCE = 1
BACKWARD_FORCE = -1
RIGHT_FORCE = 1
LEFT_FORCE = -1


class SimpleTerminalController:
    def __init__(self,
                 verbatim: bool = True,
                 maxmin_velocity: float = 10,
                 drive_type: airsim.DrivetrainType = airsim.DrivetrainType.ForwardOnly,
                 client: airsim.MultirotorClient = None):
        # Should this class print to terminal
        self.verbatim = verbatim
        self.DriveType = drive_type
        self.client = client
        if client is None:
            self.client = airsim.MultirotorClient()
        self.confirm_connection()
        # Segmentation setup
        self.setup_segmentation_colors()

        # Movement and constraints:
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.yaw = 0
        self.nav = None

        self.maxmin_vel = maxmin_velocity

    def confirm_connection(self):
        self.client.confirmConnection()
        self.client.enableApiControl(True)

    def setup_segmentation_colors(self):
        """
        Find all objects and make them one color
        then find the specific objects and turn them into different colors.
        :return:
        """
        self.set_bg_color(color_id=BG)
        self.change_color("segment_gate", LAND)

    def change_color(self, name, id):
        success = self.client.simSetSegmentationObjectID(name + "[\w]*", id, True)
        print("Change of color on", name, "=", success)

    def set_bg_color(self, color_id):
        alphabet = list(string.ascii_lowercase)
        for letter in alphabet:
            self.change_color(letter, color_id)

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
                                        velocity=float(args[4]), drivetrain=airsim.DrivetrainType.ForwardOnly,
                                        yaw_mode=airsim.YawMode(False, 0)).join()
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
            result = self.client.moveOnPathAsync(path, float(args[-1]), TIMEOUT,
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

    def orbit(self, args):  # name, speed, x,y
        if len(args) < 3:
            print("need at least speed parameter and iterations")
            return
        if len(args) != 4:  # Name, x,y
            target_x = float(72.38)  # X coordinate of turbine 1
            target_y = float(48.92)  # Y coordinate of turbine 1

            self.client.enableApiControl(True)
            self.client.moveToPositionAsync(x=float(36.33), y=float(24.32), z=-float(17.33),
                                            velocity=2, drivetrain=airsim.DrivetrainType.ForwardOnly,
                                            yaw_mode=airsim.YawMode(False, 0)).join()
            self.client.hoverAsync().join()
            airsim.time.sleep(2)
        else:
            target_x = float(args[3])
            target_y = float(args[4])
        speed = float(args[1])
        iterations = int(args[2])
        for i in range(iterations):
            current_pos = self.client.getMultirotorState().kinematics_estimated.position
            look_at_point = np.array([target_x, target_y])
            current_pos_np = np.array([current_pos.x_val, current_pos.y_val])
            angle = self.lookAt(look_at_point, np.array([1, 0]))
            l = look_at_point - current_pos_np
            radius = np.linalg.norm(l)
            print("Radius:", radius)
            # Have to make sure it is enabled:
            self.client.enableApiControl(True)
            self.client.rotateToYawAsync(angle, 20, 0).join()
            print(self.client.getMultirotorState().kinematics_estimated.orientation)

            self.nav = OrbitNavigator(self.client,
                                      radius=radius,
                                      altitude=float(current_pos.z_val),
                                      speed=speed,
                                      iterations=1,
                                      center=l)

            self.nav.start()
            print("Orbit ", i, "is done, climb to:", current_pos.x_val, current_pos.y_val, current_pos.z_val - radius)
            self.client.moveToPositionAsync(current_pos.x_val, current_pos.y_val, current_pos.z_val - radius, speed,
                                            10).join()

    def lookAt(self, target_pos, current_pos):
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        angle = np.arctan2(dy, dx) * 180 / np.math.pi
        return angle

    def handle_key_pressed(self, keys_to_check: list, pressed_keys: list, current_vel: float) -> float:
        new_vel = current_vel
        positive_axis_press = KeyCode.from_char(keys_to_check[0]) in pressed_keys
        negative_axis_press = KeyCode.from_char(keys_to_check[1]) in pressed_keys

        if positive_axis_press and negative_axis_press:
            return new_vel

        if positive_axis_press:
            return round(number=float(np.clip(new_vel + 1, - self.maxmin_vel, self.maxmin_vel)), ndigits=2)

        if negative_axis_press:
            return round(number=float(np.clip(new_vel - 1, - self.maxmin_vel, self.maxmin_vel)), ndigits=2)

        # nothing is pressed, smoothly lowering the value
        return round(number=float(np.clip(new_vel * 0.75, - self.maxmin_vel, self.maxmin_vel)), ndigits=2)

    @staticmethod
    def handle_rotation(keys_to_check: list, pressed_keys: list) -> float:
        positive_rotation_press = KeyCode.from_char(keys_to_check[0]) in pressed_keys
        negative_rotation_press = KeyCode.from_char(keys_to_check[1]) in pressed_keys

        if positive_rotation_press and negative_rotation_press:
            return 0
        if positive_rotation_press:
            return 20
        if negative_rotation_press:
            return -20
        return 0

    @staticmethod
    def handle_height(keys_to_check: list, pressed_keys: list, current_height: float) -> float:
        positive_axis_press = KeyCode.from_char(keys_to_check[0]) in pressed_keys
        negative_axis_press = KeyCode.from_char(keys_to_check[1]) in pressed_keys

        if positive_axis_press and negative_axis_press:
            return current_height
        if positive_axis_press:
            return current_height + 0.2
        if negative_axis_press:
            return current_height - 0.2
        return current_height

    def enter_keyboard_control(self):
        print("You entered the keyboard mode. Press 't' to return.")
        kc = KeyController()
        z = self.client.getMultirotorState().kinematics_estimated.position.z_val
        self.client.enableApiControl(True)
        while kc.listener.running:
            self.client.cancelLastTask()
            self.client.enableApiControl(True)
            keys = kc.get_key_pressed()
            if 'h' in keys:
                self.client.hoverAsync()
            else:
                quad_vel = self.client.getMultirotorState().kinematics_estimated.linear_velocity
                self.vx = self.handle_key_pressed(keys_to_check=['w', 's'], pressed_keys=keys,
                                                  current_vel=quad_vel.x_val)
                self.vy = self.handle_key_pressed(keys_to_check=['d', 'a'], pressed_keys=keys,
                                                  current_vel=quad_vel.y_val)
                z = self.handle_height(keys_to_check=['z', 'x'], pressed_keys=keys, current_height=z)
                self.yaw = self.handle_rotation(keys_to_check=['e', 'q'], pressed_keys=keys)
                print(
                    "current vel: \n vx:{0}, nvx:{1}\n vy:{2}, nvy:{3}\n vz:{4}, nvz:{5}\n".format(quad_vel.x_val,
                                                                                                   self.vx,
                                                                                                   quad_vel.y_val,
                                                                                                   self.vy,
                                                                                                   quad_vel.z_val,
                                                                                                   self.vz))
                current_pos = self.client.getMultirotorState().kinematics_estimated.position
                print("current pos: \n x:{0}, y:{1}\n z:{2}\n".format(current_pos.x_val, current_pos.y_val,
                                                                      current_pos.z_val))

                self.client.moveByVelocityZAsync(self.vx, self.vy, z, 0.1, airsim.DrivetrainType.MaxDegreeOfFreedom,
                                                 airsim.YawMode(True, self.yaw)).join()
            # self.client.moveByVelocityAsync(self.vx, self.vy, self.vz, 0.1, airsim.DrivetrainType.MaxDegreeOfFreedom,
            #                                 airsim.YawMode(True, self.yaw)).join()
            # airsim.time.sleep(0.2)
        print("'t' has been pressed and the console control is back")
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
            elif command_type.lower() == ORBIT:
                self.orbit(args)
            else:
                print("The command given is not a valid command.")

        # that's enough fun for now. let's quit cleanly
        airsim.wait_key("When ready to kill")
        self.client.enableApiControl(False)


if __name__ == '__main__':
    controller = SimpleTerminalController(maxmin_velocity=20)
    controller.run()
