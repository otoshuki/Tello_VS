from time import sleep
import tellopy
import cv2

def handler(event, sender, data, **args):
    drone = sender
    if event is drone.EVENT_FLIGHT_DATA:
        print(data)

def test():
    drone = tellopy.Tello()
    cap = cv2.VideoCapture(0)
    try:
        drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)
        drone.connect()
        drone.wait_for_connection(60.0)
        while 1:
            _, frame = cap.read()
            cv2.imshow("w", frame)
            if cv2.waitKey(1) == ord("s"): break
        drone.takeoff()
        sleep(5)
        drone.down(50)
        while 1:
            _, frame = cap.read()
            if cv2.waitKey(1) == ord("x"): break
        print("Landing!")
        drone.land()
        sleep(2)
    except Exception as ex:
        print(ex)
    finally:
        drone.quit()

if __name__ == '__main__':
    test()
