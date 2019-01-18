'''
File: movement.py
Function: Run movement commands in AUV
Note: Change code between ARM and DISARM
'''
import time
# Import mavutil
from pymavlink import mavutil

# Create Connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands

def wait_heartbeat(m):
    print("Wait for PX4 Heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from PX4 (system %u component %u)" % (m.target_system, m.target_system))

wait_heartbeat(master)

# Create a function to send RC values
# More information about Joystick channels
# here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs
def set_rc_channel_pwm(id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if id < 1:
        print("Channel does not exist.")
        return

    # We only have 8 channels
    #http://mavlink.org/messages/common#RC_CHANNELS_OVERRIDE
    if id < 9:
        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[id - 1] = pwm
        master.mav.rc_channels_override_send(
            master.target_system,                # target_system
            master.target_component,             # target_component
            *rc_channel_values)

# Choose a mode
mode = 'STABILIZE'

# Check if mode is available
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    exit(1)

# Get mode ID
mode_id = master.mode_mapping()[mode]
# Set new mode
# master.mav.command_long_send(
#    master.target_system, master.target_component,
#    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
#    0, mode_id, 0, 0, 0, 0, 0) or:
# master.set_mode(mode_id) or:
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

# Check ACK
ack = False
while not ack:
    # Wait for ACK command
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()

    # Check if command in the same in `set_mode`
    if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
        continue

    # Print the ACK result !
    print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    break


def movement_duration(id, pwm, length):
    counter = 0
    print("Moving on channel %d on %d PWM for %d seconds" % (id, pwm, length))                 # RC channel list, in microseconds.
    while counter != length:
        set_rc_channel_pwm(id, pwm)
        wait_heartbeat(master)
        counter += 1

# http://mavlink.org/messages/common#MAV_CMD_COMPONENT_ARM_DISARM
# Arm
# master.arducopter_arm() or
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)


"""
CHEAT SHEET
Throttle Channel = 3
Yaw Channel = 4
Forward Channel = 5
Lateral Channel = 6

Default PWM Range = 1500
"""
#Movement script
time.sleep(25)
movement_duration(5, 2000, 10)
time.sleep(2)
movement_duration(4, 2000, 2)
time.sleep(2)



# Disarm
# master.arducopter_disarm() or
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0
)
