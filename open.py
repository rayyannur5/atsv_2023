import cv2

cap  = cv2.VideoCapture('my_video-5.mkv')

while True:
    ret, frame = cap.read()

    # if not ret:
    #     break
    
    cv2.imshow('image', frame )
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()