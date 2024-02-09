import cv2

# Replace the range with the number of video devices you have, e.g., range(4) if you have /dev/video0 to /dev/video3
for index in range(0, 4):
    cap = cv2.VideoCapture(index)
    if cap is None or not cap.isOpened():
        print(f"Unable to open video source: {index}")
    else:
        ret, frame = cap.read()
        if ret:
            print(f"Video source {index} is working.")
            # Optionally show the frame to verify the correct camera
            cv2.imshow(f"Frame from video source {index}", frame)
            cv2.waitKey(500)  # Show the frame for 500 ms
        else:
            print(f"Video source {index} is not working.")
        cap.release()

cv2.destroyAllWindows()
