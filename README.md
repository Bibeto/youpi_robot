# youpi_robot

The project is based on 3DOFs robot who moves with the use of stepper motors

All the calculations for inverse and forward kinematics was done manually. So the code includes only the final equations 

Description of all tasks done : 

Building a robotic arm Youpi which catches objects in movement on a conveyor

In this project we've done four mains parts : 

*Hardware conceptualization: In this part we've made a 3D model of a conveyor on #SolidWorks and printed it using a 3D printer and #Ultimaker_Cura.

*Electronic conceptualization : In this section, we used a CNC shield and 3 drivers A4988 to control the stepper motors responsible for the movements. 

*Conversion to Cartesian coordinates: Using a #Raspberry_Pi, we convert the polar coordinates(the angles) to Cartesian coordinates (x, y, z)

*Vision system : In this part, we used an android app on a smartphone connected to the #Raspberry_Pi through USB, to make use of its reliable camera.

The system detects the movement of objects on the conveyor using #OpenCv in #Python, catches it using an aspirator and removes it from the conveyor.

A demo of the robot in action: 

https://user-images.githubusercontent.com/43536377/198924615-d4780ce5-fd33-4a9c-98b4-058771c8607b.mp4

