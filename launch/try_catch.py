import dronekit
import socket
import exceptions

try:
	dronekit.connect('/dev/serial0', heartbeat_timeout=15,baud=57600)

#bad TCP connection
except socket.error:
	print "no server exists"

#Bad TTY connection
except exceptions.OSError as e:
	print "no serial exists"

#API Error
except dronekit.APIException:
	print 'Timeout'

#other error
except:
	print 'some other error' 
