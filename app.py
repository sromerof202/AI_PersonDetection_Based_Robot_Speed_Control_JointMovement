# -*- coding: utf-8 -*-
import cv2
import torch
import time
from lib64 import jkrc

# Initialize YOLO model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

# Check if CUDA is available and if so, use it
if torch.cuda.is_available():
    model = model.to('cuda')

# Initialize camera
cap = cv2.VideoCapture(0)  # Adjust '0' to your camera ID
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Set the width to 640 pixels
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Set the height to 480 pixels
if not cap.isOpened():
    print("Unable to open camera")
    exit(1)

# Robot control constants
PI = 3.1415926
ABS = 0
joint_pos1 = [PI/20, PI/17, PI/130, PI/4, 0, 0]
joint_pos2 = [1, 0.6, 1, 1, 1, 1]  
joint_pos3 = [1, 1, 1, 1, 1, 1]  
joint_pos4 = [1, 1.5, 1, 1, 1, 1]  

# Connect to robot
try:
    robot = jkrc.RC("192.168.0.124")  # Adjust IP address
    robot.login()
    robot.power_on()
    robot.enable_robot()
except Exception as e:
    print(f"Failed to connect to robot: {e}")
    exit(1)

def detect_person(frame):
    # Preprocess frame
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_rgb = torch.from_numpy(frame_rgb).float().permute(2, 0, 1).unsqueeze(0) / 255.0

    # Move the frame to the GPU if CUDA is available
    if torch.cuda.is_available():
        frame_rgb = frame_rgb.to('cuda')

    results = model(frame_rgb)
    # Find the maximum class score and its corresponding class for each anchor box
    class_scores, class_ids = torch.max(results[0, :, 5:], dim=1)
    print(f"Class IDs: {class_ids}")  # Debug line
    print(f"Class Scores: {class_scores}")  # Debug line
    # Check for persons; class ID for persons is 0
    persons = class_ids[(class_ids == 0) & (class_scores > 0.5)]  # Add score threshold
    return len(persons) > 0, results[0]

# Speed control
speed = 1  # Start with maximum speed

try:
    for i in range(50):  # Loop for 50 rounds
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Check for persons in the frame
        person_detected, results = detect_person(frame)
        if person_detected:
            # Reduce speed if person detected
            speed = 0.1  # Slow down the robot to 10% of its normal speed

            # Draw bounding boxes on frame
            for *xyxy, conf, cls in results:
                if cls == 0:  # Person class
                    cv2.rectangle(frame, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 255, 0), 2)

        else:
            # Gradually increase speed if no person detected
            if speed < 1:
                speed += 0.3  # Increase speed by 30%

        # Move robot joints
        print("Moving to position...")
        if i % 4 == 0:
            robot.joint_move(joint_pos1, ABS, True, speed)
        elif i % 4 == 1:
            robot.joint_move(joint_pos2, ABS, True, speed)
        elif i % 4 == 2:
            robot.joint_move(joint_pos3, ABS, True, speed)
        else:
            robot.joint_move(joint_pos4, ABS, True, speed)

finally:
    # Clean up
    robot.logout()
    cap.release()