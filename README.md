# PS4VESE
This allows you to control a ground vehicle through a PS4 controller that is connected via bluetooth to your laptop (Station). This code is intended to work with a jetson nano that is using serial communication to talk with the VESE MKVI from trampa. This program does not allow for any control over the servo motor mounted at the front of the frame but it can easily be adapted to accomodate that feature. 

**Controller_tester.py** 4
This script allows you to read in the raw values coming from your joystick. It also tells you what axis these values are being read in on in case you would like to you a different input from the default Left and right triggers. 

**Controller.py** 

This is the main script to be run on your computer (Station), this program reads in values from the PS4 controller and sends them to our vehicle (Jetson) which then get converted to Duty Cycle Values to control the onboard DC motor. BE SURE TO MODIFY the ip adress in the script to fit the ip of your vehicle. 

**VESE_reader.py** 

This is the main script that is running on the ground vehicle. It reads in values coming from the PS4 controller and converts them to a duty cycle that is appropriate for your desired speed of the motor. 
