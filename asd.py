import numpy as np
import cv2 as cv
import serial, time

if __name__ == '__main__':
    #usb girislerinden bagli olani bulur
    try:
        ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        ser.reset_input_buffer()
    except Exception as e:
        print("Err: ", e.__class__, " hatasi olustu... Tekrar deneniyor")
        ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)
        ser.reset_input_buffer()

def testKamera(index):
    global cap
    cap = cv.VideoCapture(index)
    if cap is None or not cap.isOpened():
        return False
    else:
        return True
    
i = 0
while True:
    if testKamera(i):
        break
    else:
        print("kamera acilamadi, kamera indexi: ", i)
        i = i + 1
    time.sleep(0.5)

while True:
    #opencv picture loopu
    ret, frame = cap.read()
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    blur = cv.GaussianBlur(gray, (5, 5), 0)
    ret, thresh = cv.threshold(blur, 60, 255, cv.THRESH_BINARY_INV)
    contours, hierarchy = cv.findContours(thresh.copy(), 1, cv.CHAIN_APPROX_NONE)
    
    #kullandigim kaynak
    #http://einsteiniumstudios.com/beaglebone-opencv-line-following-robot.html
    if len(contours) > 0:
        c = max(contours, key=cv.contourArea)
        M = cv.moments(c)
        
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        
        cv.line(frame, (cx, 0), (cx, 720), (255, 0, 0) , 1)
        cv.line(frame, (0, cy), (1280, cy), (255, 0 ,0), 1)
        
        cv.drawContours(frame, contours, -1, (0, 255, 0), 1)
        
        print(cx)
        
        #cx degeri 650 olmasina ragmen if statementlerden sadece 2 degeri geliyor
        #syntax hatasi olabilir?
        #encoder olarak latin-1 kullandim cunku utf-8 "UnicodeDecodeError: 'utf-8' codec can't decode byte in position: invalid continuation byte" veya "invalid start byte" veya 
        #herhangi bir fark var mi?
        if cx >= 740:
            ser.write(str(1).encode('latin-1'))

        if cx < 740 and cx > 340:
            ser.write(str(2).encode('latin-1'))

        if cx <= 340:
            ser.write(str(3).encode('latin-1'))
    else:
        ser.write(str(0).encode('latin-1'))
        
    cv.imshow("frame", frame)
    #arduinonun kararlastirdigi degeri al, printle
    line = ser.readline().decode('latin-1').rstrip()
    print(line)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
    
cap.release()
cv.destroyAllWindows()