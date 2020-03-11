from dronekit import connect 
#connect to the vehicle
vehicle = connect('/dev/serial0', wait_ready= True,baud=57600)
