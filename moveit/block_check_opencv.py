import cv2
import numpy as np
from pymycobot.mycobot import MyCobot
import time

camera_cor = [140, 80, 330, 180, 0, 0]
mc = None



def detect_color(image, color_range):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_bound = np.array(color_range[0], dtype=np.uint8)
    upper_bound = np.array(color_range[1], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    return mask

def draw_bounding_box(image, mask, color):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cv2.contourArea(cnt) > 100:  # Minimum size area to consider
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)

def main():
    global mc
    port = 'COM3'
    baudrate = 115200
    mc = MyCobot(port, baudrate)
    mc.send_coords(camera_cor, 70, 1)
    time.sleep(1)   
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Define color ranges in HSV
        red_range = ([0, 150, 50], [10, 255, 255])  # Red range
        yellow_range = ([20, 100, 100], [30, 255, 255])  # Yellow range
        blue_range = ([100, 100, 50], [135, 255, 255])  # Blue range
        purple_range = ([130, 100, 50], [150, 255, 255])  # Purple range

        # Create masks for each color
        red_mask = detect_color(frame, red_range)
        yellow_mask = detect_color(frame, yellow_range)
        blue_mask = detect_color(frame, blue_range)
        purple_range = detect_color(frame, purple_range)

        # Draw bounding boxes
        draw_bounding_box(frame, red_mask, (0, 0, 255))   # Red bounding box
        draw_bounding_box(frame, yellow_mask, (0, 255, 255))  # Yellow bounding box
        draw_bounding_box(frame, blue_mask, (255, 0, 0))  # Blue bounding box
        draw_bounding_box(frame, purple_range, (255, 0, 255))  # Purple bounding box

        cv2.imshow('Frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
