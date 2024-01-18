import cv2

def main():
    print('Starting video capture from image_collector.')

    # Initialize video capture from the default camera
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open video capture device.")
        return

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Error: Failed to capture frame.")
            break

        # Display the resulting frame
        cv2.imshow('Frame', frame)

        # Press 'q' on the keyboard to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the video capture object
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()