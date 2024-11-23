import cv2
import time

def process_image(image):
    print("Processing image...")
    cv2.imshow('Processed Image', image)
    cv2.waitKey(1) 


def object_detection(image):
    print("object detection...")


def depth_mapping(image):
    print("Depth mapping...")


def capture_image():
    cap = cv2.VideoCapture(0) 
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to grab frame.")
            break
        
        process_image(frame)

        time.sleep(1)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    capture_image()
