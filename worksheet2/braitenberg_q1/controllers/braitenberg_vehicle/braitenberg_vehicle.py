"""braitenberg_vehicle controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

TIMESTEP = 64
MAX_SPEED = 10

def run_robot(robot):
    # Enable motors
    wheels = []
    wheelNames = ["wheel1","wheel2","wheel3","wheel4"]
    
    for i in range(len(wheelNames)):
        wheels.append(robot.getDevice(wheelNames[i]))
        wheels[i].setPosition(float('inf'))
        wheels[i].setVelocity(0.0)
        
    # Enable light sensors
    light_sensors = []
    light_sensor_names = ['ls_left', 'ls_right']
    for i in range(len(light_sensor_names)):
        light_sensors.append(robot.getDevice(light_sensor_names[i]))
        light_sensors[i].enable(TIMESTEP)

    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(TIMESTEP) != -1:
        # Read the sensors:
        left_light_val = light_sensors[0].getValue()
        right_light_val = light_sensors[1].getValue()

        # Process sensor data here.
        left_speed = -MAX_SPEED
        right_speed = -MAX_SPEED
    
        # Enter here functions to send actuator commands, like:
        # Wheel 1,2 are on the right side
        wheels[0].setVelocity(right_speed)
        wheels[1].setVelocity(right_speed)
        
        # Wheel 3,4 are on the left side
        wheels[2].setVelocity(left_speed)
        wheels[3].setVelocity(left_speed)
    
# Enter here exit cleanup code.

if __name__ == "__main__":
    # create the Robot instance.
    my_robot = Robot()
    run_robot(my_robot)
    
