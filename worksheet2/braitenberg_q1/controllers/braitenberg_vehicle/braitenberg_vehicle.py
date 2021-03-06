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
        
        self.signs = {'inverse': -1.0, 'proportional': 1.0}
        
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
            
    def transfer_function_double_connection(self, input1, input2, attraction_type):
        output1 = input1/self.normalization_factor
        output2 = input2/self.normalization_factor
        
        if attraction_type == 'inverse':
            output1 = self.MAX_SPEED - output1
            output2 = self.MAX_SPEED - output2
        
        return (output1, output2)
            
    def run_robot(self, transfer_function):
        # Perform simulation steps until Webots is stopped by the controller
        while self.robot.step(self.TIMESTEP) != -1:
            # Read the sensors:
            if transfer_function == 'linear':
                ls_val = self.sensors['ls_center'].getValue()/100.0
            else:
                # Robot doesn't like extremes. Moves very quickly if too much
                # or not enough light.
                ls_val = self.sensors['ls_center'].getValue()/100.0
                if ls_val < 3 or ls_val > 7:
                    ls_val = self.MAX_SPEED
            
            print(ls_val)
            
            self.wheels['wheel3'].setVelocity(ls_val)

if __name__ == "__main__":
    # Create Braitenberg Vehicle
    wheel_names = ["wheel3"]
    light_sensor_names = ['ls_center']
    braitenberg_vehicle = BraitenbergVehicle(wheel_names, light_sensor_names)
    
    # Define motivational states
    transfer_function_linear = 'linear'
    transfer_function_hate_extrema = 'hate_extrema'
    
    braitenberg_vehicle.run_robot(transfer_function_hate_extrema)
    
