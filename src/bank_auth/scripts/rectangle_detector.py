#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from std_msgs.msg import Bool

login_status = False
deteksi_status = True 

def callback():
    pass

def detect_black_rectangle():
    pub = rospy.Publisher('deteksi_login', Bool, queue_size=10)
    rate = rospy.Rate(10)

    cap = cv2.VideoCapture(2) 
    if not cap.isOpened():
        rospy.logerr("Gagal membuka kamera!")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    rospy.loginfo("Sedang mendeteksi...")

    cv2.namedWindow("Trackbars")
    
    cv2.createTrackbar("LH", "Trackbars", 0, 255, callback)
    cv2.createTrackbar("LS", "Trackbars", 0, 255, callback)
    cv2.createTrackbar("LV", "Trackbars", 0, 255, callback)
    cv2.createTrackbar("UH", "Trackbars", 180, 180, callback)
    cv2.createTrackbar("US", "Trackbars", 255, 255, callback)
    cv2.createTrackbar("UV", "Trackbars", 30, 255, callback)

    detected = False

    try:
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                rospy.logerr("Gagal membaca frame dari kamera!")
                break

            lh = cv2.getTrackbarPos("LH", "Trackbars")
            ls = cv2.getTrackbarPos("LS", "Trackbars")
            lv = cv2.getTrackbarPos("LV", "Trackbars")
            uh = cv2.getTrackbarPos("UH", "Trackbars")
            us = cv2.getTrackbarPos("US", "Trackbars")
            uv = cv2.getTrackbarPos("UV", "Trackbars")

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            lower_black = np.array([lh, ls, lv])
            upper_black = np.array([uh, us, uv])
            mask = cv2.inRange(hsv, lower_black, upper_black)

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            detected = False
            for cnt in contours:
                epsilon = 0.02 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                if len(approx) == 4:  
                    x, y, w, h = cv2.boundingRect(approx)
                    aspect_ratio = float(w) / h
                    if 0.8 < aspect_ratio < 1.2 and w > 50 and h > 50:  
                        detected = True
                        cv2.drawContours(frame, [approx], -1, (0, 255, 0), 3)
                        cv2.putText(frame, "Black Rectangle", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        rospy.loginfo("Persegi panjang hitam terdeteksi.")
                        pub.publish(True)
                        break

            if not detected and not rospy.is_shutdown():
                rospy.loginfo("Tidak ada persegi panjang hitam.")
                pub.publish(False)

            cv2.imshow("Frame", frame)
            cv2.imshow("Mask", mask)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if detected:
                break


            rate.sleep()

    finally:
        cap.release()
        cv2.destroyAllWindows()
        rospy.loginfo("Sistem dihentikan.")

if __name__ == '__main__':
    try:
        rospy.init_node('detector_node', anonymous=True)
        detect_black_rectangle()
    except rospy.ROSInterruptException:
        pass
