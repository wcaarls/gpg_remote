#!/usr/bin/env python

import io
import socket
import select
import struct
import time
import gopigo3

GPG = gopigo3.GoPiGo3()

server_socket = socket.socket()
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(('0.0.0.0', 8002))
server_socket.listen(0)

while True:
    print "Awaiting connection"
    
    connection = server_socket.accept()[0]
    
    print "Connected"
    
    try:
        while True:
            ready = select.select([connection], [], [], 0.010)[0] # 10ms timeout
            
            if ready:
                sz = struct.unpack('<L', connection.recv(struct.calcsize('<L')))[0]
                if not sz:
                    break
                    
                msg = connection.recv(sz)
                msg = struct.unpack('<lll', msg)
                
                # Apply commands
                GPG.set_motor_dps(GPG.MOTOR_LEFT, msg[0])
                GPG.set_motor_dps(GPG.MOTOR_RIGHT, msg[1])
                GPG.set_servo(GPG.SERVO_1, msg[2])
                
                
            # Construct status
            msg = struct.pack('<ll', GPG.get_motor_encoder(GPG.MOTOR_LEFT)*GPG.MOTOR_TICKS_PER_DEGREE, GPG.get_motor_encoder(GPG.MOTOR_RIGHT)*GPG.MOTOR_TICKS_PER_DEGREE)
            msg = struct.pack('<L', len(msg)) + msg
            
            connection.send(msg)
    except:
        print "Error"
        pass
    finally:
        try:
            connection.close()
        except:
            pass

GPG.reset_all()
server_socket.close()
