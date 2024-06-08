import mediapipe as mp
import cv2
import numpy as np
import serial
import math
ser = serial.Serial("COM4", "115200")

def cal_angle(a,b,c):
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)
    radian = np.arctan2(c[1]-b[1],c[0]-b[0]) - np.arctan2(a[1]-b[1],a[0]-b[0])

    angle = np.abs(radian*180.0/np.pi)

    if(angle > 180):
        angle = 360 - angle
    return angle


cap = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)

mp_pose = mp.solutions.pose

isMoveDone = True


with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    while cap.isOpened():
        ret, image = cap.read()
        image_h, image_w, _ = image.shape
        results = pose.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

        if results.pose_landmarks :
            left_shoulder = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER]
            left_wrist = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST]
            left_elbow = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW]
            left_index = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_INDEX]
            left_pinky = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_PINKY]

            midpoint_x = (left_index.x + left_pinky.x) / 2
            midpoint_y = (left_index.y + left_pinky.y) / 2

            

            shoulderl_pos = (int(left_shoulder.x * image_w), int(left_shoulder.y * image_h))
            handl_pos = (int(left_wrist.x * image_w), int(left_wrist.y * image_h))
            elbowl_pos = (int(left_elbow.x * image_w), int(left_elbow.y * image_h))
            midpoint_pos = (int(midpoint_x * image_w), int(midpoint_y * image_h))



            cv2.circle(image,shoulderl_pos, 10, (255,0,0),-1)
            cv2.circle(image,handl_pos, 10, (0,255,0),-1)
            cv2.circle(image,elbowl_pos, 10, (0,0,255),-1)
            cv2.circle(image,midpoint_pos, 10, (255,0,0),-1)

            
            cv2.line(image,shoulderl_pos,elbowl_pos,(255,0,0),3)
            cv2.line(image,elbowl_pos,handl_pos,(0,255,0),3)
            cv2.line(image,midpoint_pos,handl_pos,(0,255,0),3)


            m = (elbowl_pos[1] - shoulderl_pos[1]) / (elbowl_pos[0] - shoulderl_pos[0])

            # 직선과 x축(지면)과의 각도 계산
            angle_fir = np.arctan(m) * 180 / np.pi

            # 지면과의 기울기
            ground_slope = 0  # 지면이 수평이므로 기울기는 0

            # 직선과 지면 사이의 각도 계산 후 반전하여 표기
            angle_fir = max(abs(angle_fir - np.arctan(ground_slope) * 180 / np.pi), 1)

            angle_sec = max(270 - cal_angle(shoulderl_pos, elbowl_pos, handl_pos), 1)
            angle_thr = max(270-  cal_angle(elbowl_pos, handl_pos, midpoint_pos), 1)


            flipped_image = cv2.flip(image, 1)
            cv2.putText(flipped_image, "{:.2f}".format(angle_fir), (flipped_image.shape[1] - shoulderl_pos[0], shoulderl_pos[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 3)
            cv2.putText(flipped_image, "{:.2f}".format(angle_sec), (flipped_image.shape[1] - elbowl_pos[0], elbowl_pos[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 3)
            cv2.putText(flipped_image, "{:.2f}".format(angle_thr), (flipped_image.shape[1] - handl_pos[0], handl_pos[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 3)


            command = f"90,{int(angle_fir)},{int(angle_sec)},{int(angle_thr)}d"
            command_bytes = command.encode('utf-8')

            if(isMoveDone):
                ser.write(command_bytes)
                print(command)
                isMoveDone = False
            else :
                print("skip")

            if ser.read() == b'd' :
                print("process done") 
                isMoveDone = True
            else :
                print("wait")


            cv2.imshow("my cam", flipped_image)

            key = cv2.waitKey(1)
            if(key == ord('q') or key == 27):
                command = f"90,90,90,90d"
                command_bytes = command.encode('utf-8')
                ser.write(command_bytes)
                break
            
cap.release()
cv2.destroyAllWindows()
ser.close()

            