#!/usr/bin/env python3
import zmq

context = zmq.Context()
socket = context.socket(zmq.PUSH)
socket.connect("tcp://frcvision.local:5555")

while True:
    input("Press enter to save a frame")
    socket.send_string("write_frame")
