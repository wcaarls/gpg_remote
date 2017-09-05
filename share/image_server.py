#!/usr/bin/env python

# http://picamera.readthedocs.io/en/release-1.9/recipes1.html#capturing-to-a-network-stream

import io
import socket
import struct
import time
import picamera

camera = picamera.PiCamera()
camera.resolution = (640, 480)
# Start a preview and let the camera warm up for 2 seconds
camera.start_preview()
time.sleep(2)

# Start a socket listening for connections on 0.0.0.0:8000 (0.0.0.0 means
# all interfaces)
server_socket = socket.socket()
server_socket.bind(('0.0.0.0', 8000))
server_socket.listen(0)

while True:
    # Accept a single connection and make a file-like object out of it
    connection = server_socket.accept()[0].makefile('rb')
    try:
        stream = io.BytesIO()
        for foo in camera.capture_continuous(stream, 'jpeg', use_video_port=True):
            # Write the length of the capture to the stream and flush to
            # ensure it actually gets sent
            connection.write(struct.pack('<L', stream.tell()))
            connection.flush()
            # Rewind the stream and send the image data over the wire
            stream.seek(0)
            connection.write(stream.read())
            # Reset the stream for the next capture
            stream.seek(0)
            stream.truncate()
    except:
        pass
    finally:
        try:
            connection.close()
        except:
            pass

server_socket.close()
