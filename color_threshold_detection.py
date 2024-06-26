import cv2
import numpy as np

def nothing(x):
    pass

# Tạo một cửa sổ
cv2.namedWindow("Color Detection")

# Tạo các thanh trượt cho các giá trị màu sắc HSV
cv2.createTrackbar("Lower Hue", "Color Detection", 0, 179, nothing)
cv2.createTrackbar("Lower Saturation", "Color Detection", 0, 255, nothing)
cv2.createTrackbar("Lower Value", "Color Detection", 0, 255, nothing)
cv2.createTrackbar("Upper Hue", "Color Detection", 179, 179, nothing)
cv2.createTrackbar("Upper Saturation", "Color Detection", 255, 255, nothing)
cv2.createTrackbar("Upper Value", "Color Detection", 255, 255, nothing)

# Bắt đầu quay video từ webcam
webcam = cv2.VideoCapture(0)

while True:
    _, frame = webcam.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Đọc các giá trị từ thanh trượt
    lh = cv2.getTrackbarPos("Lower Hue", "Color Detection")
    ls = cv2.getTrackbarPos("Lower Saturation", "Color Detection")
    lv = cv2.getTrackbarPos("Lower Value", "Color Detection")
    uh = cv2.getTrackbarPos("Upper Hue", "Color Detection")
    us = cv2.getTrackbarPos("Upper Saturation", "Color Detection")
    uv = cv2.getTrackbarPos("Upper Value", "Color Detection")

    # Đặt ngưỡng màu
    lower_bound = np.array([lh, ls, lv])
    upper_bound = np.array([uh, us, uv])
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # Áp dụng mask lên hình ảnh gốc
    result = cv2.bitwise_and(frame, frame, mask=mask)

     # Tìm các contour
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 1000:  # Ngưỡng để lọc các đối tượng nhỏ
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f"Area: {int(area)}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 2)


    # Hiển thị kết quả
    cv2.imshow("Color Detection", result)
    cv2.imshow("Original", frame)

    # Nhấn 'q' để thoát
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

webcam.release()
cv2.destroyAllWindows()
