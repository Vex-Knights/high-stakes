# Library imports
from vex import *

E = 2.7183

def pretty_print_dict(d, indent=0):
    spacing = " " * indent  # Create indent spacing
    output = ""  # Initialize output string
    for key, value in d.items():
        if isinstance(value, dict):  # If value is another dictionary, process recursively
            output += spacing + key + ":\n"
            output += pretty_print_dict(value, indent + 4)  # Increase indent for nested dicts
        elif isinstance(value, list):  # If value is a list, format each item
            output += spacing + key + ":\n"
            for item in value:
                output += spacing + "    - " + str(item) + "\n"
        else:
            output += spacing + key + ": " + str(value) + "\n"
    return output  # Return the constructed string

# Brain and controller setup
controller = Controller(PRIMARY)
brain = Brain()

intake = {
    'intake': Motor(Ports.PORT20, GearSetting.RATIO_18_1, True),
    'elevator': Motor(Ports.PORT2, GearSetting.RATIO_18_1, True),
}

# Robot configuration code
right_drive = [
    Motor(Ports.PORT15, GearSetting.RATIO_18_1, True),
    Motor(Ports.PORT17, GearSetting.RATIO_18_1, True),
]
left_drive = [
    Motor(Ports.PORT18, GearSetting.RATIO_18_1, False),
    Motor(Ports.PORT13, GearSetting.RATIO_18_1, False),
]

stakeholder = Pneumatics(brain.three_wire_port.a)
arm_sm = Pneumatics(brain.three_wire_port.b)

def pow(base, exponent):
    res = 1
    for _ in range(exponent):
        res *= base
    return res

def speedFunction (x):
    # https://www.vexforum.com/t/guide-to-exponential-drive-function/58590
    # 1.2(1.043)^x - 1.2 + 0.2x
    # y=aln(bx)
    
    return 1.2 * pow(1.053, x) - 1.2 + 0.2 * x

robotState = {
    'right_drive_train': {
        'on': False,
        'speed': 0,
        'direction': FORWARD,
    },
    'left_drive_train': {
        'on': False,
        'speed': 0,
        'direction': FORWARD,
    },
    'intake': {
        'toggle': {
            'pressed': False,
        },
        'on': False,
        'direction': FORWARD,
        'speed': 300, # 130 optimal
        'reverse': {
            'pressed': False,
        }
    },
    'stakeholder': {
        'pressed': False,
        'on': False,
    },
    'arm_sm': {
        'pressed': False,
        'on': False,
    }
}

while True:
    # Reset the state
    robotState = {
        'right_drive_train': {
            'on': False,
            'speed': 0,
            'direction': FORWARD,
        },
        'left_drive_train': {
            'on': False,
            'speed': 0,
            'direction': FORWARD,
        },
        'intake': {
            'toggle': {
                'pressed': robotState['intake']['toggle']['pressed'],
            },
            'on': robotState['intake']['on'],
            'direction': robotState['intake']['direction'],
            'speed': robotState['intake']['speed'],
            'reverse': {
                'pressed': robotState['intake']['reverse']['pressed'],
            }
        },
        'stakeholder': {
            'pressed': robotState['stakeholder']['pressed'],
            'on': robotState['stakeholder']['on'],
        },
        'arm_sm': {
            'pressed': robotState['arm_sm']['pressed'],
            'on': robotState['arm_sm']['on'],
        }

    }

    if controller.buttonB.pressing():
        if not robotState['stakeholder']['pressed']:
            robotState['stakeholder']['pressed'] = True
            robotState['stakeholder']['on'] = not robotState['stakeholder']['on']
    else:
        robotState['stakeholder']['pressed'] = False

    if controller.buttonL2.pressing():
        if not robotState['arm_sm']['pressed']:
            robotState['arm_sm']['pressed'] = True
            robotState['arm_sm']['on'] = not robotState['arm_sm']['on']
    else:
        robotState['arm_sm']['pressed'] = False

    if controller.buttonR2.pressing():
        if not robotState['intake']['toggle']['pressed']:
            robotState['intake']['toggle']['pressed'] = True
            robotState['intake']['on'] = not robotState['intake']['on']
    else:
        robotState['intake']['toggle']['pressed'] = False

    if controller.buttonA.pressing():
        if not robotState['intake']['reverse']['pressed']:
            robotState['intake']['reverse']['pressed'] = True
            if robotState['intake']['direction'] == FORWARD:
                robotState['intake']['direction'] = REVERSE
            else:
                robotState['intake']['direction'] = FORWARD
    else:
        robotState['intake']['reverse']['pressed'] = False

    if robotState['intake']['direction'] == FORWARD:
        if controller.buttonR1.pressing():
            if robotState['intake']['speed'] < 300:
                robotState['intake']['speed'] += 20
        if controller.buttonL1.pressing():
            # min value -300
            if robotState['intake']['speed'] > -300:
                if robotState['intake']['speed'] >= 20:
                    robotState['intake']['speed'] -= 20
    else:
        if controller.buttonL1.pressing():
            if robotState['intake']['speed'] > -300:
                robotState['intake']['speed'] -= 20
        if controller.buttonR1.pressing():
            # min value -300
            if robotState['intake']['speed'] < 300:
                robotState['intake']['speed'] += 20

    if robotState['intake']['speed'] < 0:
        if robotState['intake']['direction'] == FORWARD:
            robotState['intake']['direction'] = REVERSE
        else:
            robotState['intake']['direction'] = FORWARD

    # This value is used later on, so no ABS
    left_axis = controller.axis1.position() # controls rotatioon
    right_axis = controller.axis3.position()

    # Handle throttle
    if right_axis > 0:
        robotState['left_drive_train']['direction'] = FORWARD
        robotState['right_drive_train']['direction'] = FORWARD
    elif right_axis < 0:
        robotState['left_drive_train']['direction'] = REVERSE
        robotState['right_drive_train']['direction'] = REVERSE
    
    # Motor on vs off
    if right_axis != 0:
        robotState['left_drive_train']['on'] = True
        robotState['right_drive_train']['on'] = True
        
        robotState['left_drive_train']['speed'] = abs(right_axis)
        robotState['right_drive_train']['speed'] = abs(right_axis)

        # Adjust speed based on arcade controls (assuming both right_axis and left_axis are used)

        wheel_dcl = (( (100 - abs(left_axis))/100 ) )

        if wheel_dcl < 0.4:
            wheel_dcl = 0.4

        if left_axis > 0: # right
            robotState['right_drive_train']['speed'] *= (1 + (abs(left_axis)/100))
            robotState['left_drive_train']['speed'] *= wheel_dcl

        elif left_axis < 0: # left
            robotState['left_drive_train']['speed'] *= (1 + (abs(left_axis)/100))
            robotState['right_drive_train']['speed'] *= wheel_dcl
    else:
        if left_axis > 0:
            robotState['left_drive_train']['direction'] = REVERSE
            robotState['right_drive_train']['direction'] = FORWARD
        elif left_axis < 0:
            robotState['left_drive_train']['direction'] = FORWARD
            robotState['right_drive_train']['direction'] = REVERSE
               
        robotState['left_drive_train']['speed'] = abs(left_axis)
        robotState['right_drive_train']['speed'] = abs(left_axis)

        if left_axis == 0:
            robotState['left_drive_train']['on'] = False
            robotState['right_drive_train']['on'] = False
        else:
            robotState['left_drive_train']['on'] = True
            robotState['right_drive_train']['on'] = True



    # Read state to motor
    if robotState['left_drive_train']['on']:
        for motor in left_drive:
            motor.spin(robotState['left_drive_train']['direction'], speedFunction(robotState['left_drive_train']['speed']))
    else:
        for motor in left_drive:
            motor.stop()

    if robotState['right_drive_train']['on']:
        for motor in right_drive:
            motor.spin(robotState['right_drive_train']['direction'], speedFunction(robotState['right_drive_train']['speed']))
    else:
        for motor in right_drive:
            motor.stop()

    if robotState['intake']['on']:
        intake['intake'].spin(robotState['intake']['direction'], robotState['intake']['speed'])
        intake['elevator'].spin(robotState['intake']['direction'], robotState['intake']['speed'])
    else:
        intake['intake'].stop()
        intake['elevator'].stop()

    if robotState['stakeholder']['on']:
        stakeholder.open()
    else:
        stakeholder.close()

    if robotState['arm_sm']['on']:
        arm_sm.open()
    else:
        arm_sm.close()

    # Apply motor actions based on robot state
    # Clear the screen and print the current robot state
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)    

    print(pretty_print_dict(robotState))


    # Brief delay to allow text to print without tearing
    wait(100, MSEC)
