#!/usr/bin/env python3
import zmq

context = zmq.Context()
socket = context.socket(zmq.PUSH)
socket.connect("tcp://frcvision.local:5555")

while True:
    pipeline = input("Pipeline name:")
    socket.send_multipart([b"set_pipeline", pipeline.encode()])