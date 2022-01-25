"""braitenberg_vehicle controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot


class BraitenbergVehicle:
    def __init__(self, wheel_names, sensor_names):
        self.wheel_names = wheel_names
        self.sensor_names = sensor_names
        self.wheels = {}
        self.sensors = {}
        
        self.TIMESTEP = 64
        self.MAX_SPEED = 10
        
        self.initial_wheel_position = float('inf')
        self.initial_wheel_velocity = 0.0
        
        # Scale factor for normalizing sensor measurements
        self.normalization_factor = 100.0
        
        self.robot = Robot()
        
        self.setup_robot(self.wheel_names, self.sensor_names)
        
        self.signs = {'attract': -1.0, 'repel': 1.0}
        
    def setup_robot(self, wheel_names, sensor_names):        
        # Enable motors
        for i in range(len(wheel_names)):
            self.wheels[wheel_names[i]] = self.robot.getDevice(wheel_names[i])
            self.wheels[wheel_names[i]].setPosition(self.initial_wheel_position)
            self.wheels[wheel_names[i]].setVelocity(self.initial_wheel_velocity)
            
        # Enable sensors
        for i in range(len(sensor_names)):
            self.sensors[sensor_names[i]] = self.robot.getDevice(sensor_names[i])
            self.sensors[sensor_names[i]].enable(self.TIMESTEP)
            
    def transfer_function_double_connection(self, input1, input2, sign=-1.0):
        output1 = sign*input1/self.normalization_factor
        output2 = sign*input2/self.normalization_factor
        
        return (output1, output2)
            
    def run_robot(self, connection_type, attraction_type):
        # Perform simulation steps until Webots is stopped by the controller
        while self.robot.step(self.TIMESTEP) != -1:
            # Read the sensors:
            left_light_val = self.sensors['ls_left'].getValue()
            right_light_val = self.sensors['ls_right'].getValue()
    
            # Process sensor data here.
            if connection_type == 'direct':
                left_speed, right_speed = self.transfer_function_double_connection(
                                           left_light_val, right_light_val, self.signs[attraction_type])
            elif connection_type == 'cross':
                # For cross connection, swap connection sides.
                right_speed, left_speed = self.transfer_function_double_connection(
                                           left_light_val, right_light_val, self.signs[attraction_type])
            else:
                raise ValueError(f'Unsupported connection type {connection_type}')
            
            print(left_speed, right_speed)
        
            # Enter here functions to send actuator commands, like:
            # Wheel 1 and 2 is on the right side
            self.wheels['wheel1'].setVelocity(right_speed)
            # self.wheels['wheel2'].setVelocity(right_speed)
            
            # Wheel 3,4 are on the left side
            self.wheels['wheel3'].setVelocity(left_speed)
            # self.wheels['wheel4'].setVelocity(left_speed)

if __name__ == "__main__":
    # Create Braitenberg Vehicle
    wheel_names = ["wheel1", "wheel2", "wheel3", "wheel4"]
    wheel_names = ["wheel1", "wheel3"]
    light_sensor_names = ['ls_left', 'ls_right']
    braitenberg_vehicle = BraitenbergVehicle(wheel_names, light_sensor_names)
    
    braitenberg_vehicle.run_robot(connection_type='cross', attraction_type='repel')
    
