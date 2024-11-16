import cv2
import time

def process_image(image):
    # Simulate image processing (e.g., object detection, depth mapping)
    print("Processing image...")
    cv2.imshow('Processed Image', image)
    cv2.waitKey(1)  # Display image for a short time

def capture_image():
    # Initialize camera capture (assumes webcam or camera module is connected)
    cap = cv2.VideoCapture(0)  # OpenCV uses 0 for the default camera
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to grab frame.")
            break
        
        # Process the captured frame
        process_image(frame)

        # Simulate frame processing delay
        time.sleep(1)

    # Release the camera when done
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    capture_image()
