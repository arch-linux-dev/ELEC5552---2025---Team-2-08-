import cv2
from datetime import datetime

vid_stream_url = "http://192.168.43.44:81/stream"
cap = cv2.VideoCapture(vid_stream_url)

# Grab first frame to get size
ret, frame = cap.read()
if not ret:
    print("Failed to grab first frame")
    exit()

frame_size = (frame.shape[1], frame.shape[0])
fourcc = cv2.VideoWriter_fourcc(*'XVID')
fps = 20.0
filename = datetime.now().strftime("CAPT_%Y%m%d_%H%M%S.avi")
out = cv2.VideoWriter(filename, fourcc, fps, frame_size)
if not out.isOpened():
    print("Error: VideoWriter not opened")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame grab failed, stopping...")
        break

    # Ensure 3 channels
    if len(frame.shape) == 2:
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    out.write(frame)
    cv2.imshow("ESP32-CAM", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
out.release()
cv2.destroyAllWindows()
