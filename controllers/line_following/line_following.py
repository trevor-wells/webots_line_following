from controller import Robot
# import numpy as np

#create robot
robot = Robot()

#define constants
TIME_STEP = 32
MAX_SPEED = 6.28

#initiate measured variables
distance = 0
rotation = 0

# grab motors and enable them to spin
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

#grab light sensors and activate them
ground_sensors = []
for i in range(3):
    ground_sensors.append(robot.getDevice('gs' + str(i)))
    ground_sensors[-1].enable(TIME_STEP)

#main loop   
while robot.step(TIME_STEP) != -1:

    #get intensity values from light sensors
    values = []
    for sensor in ground_sensors:
        values.append(sensor.getValue())
    
    # #calculate robot's displacement in the world
    # x_world = x_world + np.cos(alpha)* deltaX
    # y_world = y_world + np.sin(alpha) * deltaX
    
    # #calculate robot's z-rotation in the world
    # alpha = alpha + deltaomegaz
    
    print(values)
    #line-following behavior
    #go straight
    if (values[0] > 500 and values[1] < 350 and values[2] > 500):
        phildot, phirdot = MAX_SPEED, MAX_SPEED
    #stop
    elif (values[0] < 305 and values[1] < 305 and values[2] < 305):
        phildot, phirdot = 0, 0
    #turn right
    elif (values[2] < 350):                           
        phildot, phirdot = 0.25 * MAX_SPEED, -0.1 * MAX_SPEED
    #turn left
    elif (values[0] < 350):                              
        phildot, phirdot = -0.1 * MAX_SPEED, 0.25 * MAX_SPEED
    
    #set actuators based on calculations    
    left_motor.setVelocity(phildot)
    right_motor.setVelocity(phirdot)
    
    distance += ((0.0201*phildot + 0.0201*phirdot)/2) * (TIME_STEP/1000)
    rotation += ((((0.0201*phirdot - 0.0201*phildot)/0.052) * (TIME_STEP/1000))/3.141592) * 180
    # print("distance: " + str(distance))
    # print("rotation: " + str(rotation))
    
    # print("error: " + np.sqrt(x_world**2+y_world**2))
        
    
    