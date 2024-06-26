import cv2
import numpy as np
import serial
import time
import threading
import signal

shared_variable = 0
data_lock = threading.Lock()
terminate_flag = False  # Flag to control thread termination

def send_serial():
    global shared_variable, terminate_flag
    Ser = serial.Serial('/dev/ttyACM0', 115200)
    time.sleep(2)
    connected = Ser.is_open  # Check if the serial connection is open
    if connected:
        print("Serial connection is open.")
    else:
        print("Serial connection is not open. Check your serial port.")

    while not terminate_flag:
        with data_lock:
            data = str(shared_variable)
        if data != '0':
            Ser.write(data.encode())
            time.sleep(1)
        else:
            time.sleep(1)

def signal_handler(signal, frame):
    global terminate_flag
    print("Press Ctrl+C: Exiting the program...")
    terminate_flag = True

def detect_color(frame, hsv_frame, color_lower, color_upper, color_name, box_color, index):
    mask = cv2.inRange(hsv_frame, color_lower, color_upper)
    kernel = np.ones((5, 5), "uint8")
    mask = cv2.dilate(mask, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 40000:  # Ngưỡng diện tích
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
            cv2.putText(frame, f"{color_name} Colour, Area: {int(area)}", (x, y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0))
            return index
    return 0

def detection():
    global shared_variable, terminate_flag
    # Bắt đầu quay video từ webcam
    webcam = cv2.VideoCapture(0)

    # Set frame dimensions
    webcam.set(3, 640)  # Width
    webcam.set(4, 480)  # Height

    while True:
        if terminate_flag:  # Check the flag to terminate the thread
            break
        ret, frame = webcam.read()
        if not ret:
            break

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Phát hiện màu vàng
        yellow_index = detect_color(frame, hsv_frame, np.array([15, 100, 100], np.uint8), np.array([30, 255, 255], np.uint8), "Yellow", (0, 255, 255), 3)
        
        # Phát hiện màu xanh dương
        blue_index = detect_color(frame, hsv_frame, np.array([94, 80, 2], np.uint8), np.array([120, 255, 255], np.uint8), "Blue", (255, 0, 0), 2)

        # Phát hiện màu đỏ
        red_index = detect_color(frame, hsv_frame, np.array([136, 87, 111], np.uint8), np.array([180, 255, 255], np.uint8), "Red", (0, 0, 255), 1)

        with data_lock:
            if yellow_index: shared_variable = yellow_index
            elif blue_index: shared_variable = blue_index
            elif red_index: shared_variable = red_index
            else: shared_variable = 0

        # Hiển thị kết quả
        cv2.imshow("Color Detection", frame)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            terminate_flag = True  # Thiết lập biến cờ để tắt luồng

    webcam.release()
    cv2.destroyAllWindows()


def main():
    # Register a signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    t1 = threading.Thread(target=detection, args=())
    t2 = threading.Thread(target=send_serial, args=())

    t1.start()
    t2.start()

    t1.join()
    t2.join()

    print("Done!")

if __name__ == "__main__":
    main()
