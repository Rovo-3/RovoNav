from pyjoystick.sdl2 import Key, run_event_loop, stop_event_wait
import threading
import time
import sys

class StoppableThread(threading.Thread):
    def __init__(self, target=None, args=()):
        super().__init__()
        self._target = target
        self._args = args
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    def run(self):
        if self._target:
            self._target(*self._args)

def print_add(joy):
    print('Added', joy)

def print_remove(joy):
    print('Removed', joy)

def key_received(key):
    print('received', key)

def otherloop(stop_event):
    try:
        while not stop_event.stopped():
            print("kondisi")
            time.sleep(1)
    except KeyboardInterrupt:
        print("KeyboardInterrupt detected in other loop, stopping thread...")
def printData():
    try:
        while True:
            print("Printing from another thread...")
            time.sleep(1)  # Sleep for 1 second
    except KeyboardInterrupt: 
        print("get stop command")
    # print("Thread satunya")

if __name__ == '__main__':

    try:
    # Start the event loop
        printing_thread = threading.Thread(target=printData)
        printing_thread.start()
        event_loop = run_event_loop(print_add, print_remove, key_received)
        
    except KeyboardInterrupt:
        # If a KeyboardInterrupt occurs, stop the event loop
        stop_event_wait()
        printing_thread.join()
        sys.exit(0)  # Exit gracefully
    print("UDah di akhir boss")
    # stop_event = StoppableThread()
    # # joystickEventLoop = EventLoop()
    # thread1 = StoppableThread(target=otherloop, args=(stop_event,))
    # # joystick_thread = StoppableThread(target=run_event_loop, args=(print_add, print_remove, key_received, joystickEventLoop,))
    # event_loop = run_event_loop(print_add, print_remove, key_received)
    # stop_event_wait()
    # # if event_loop.is_alive():
    # #     print("Event loop is still running.")
    # # else:
    # #     print("Event loop has stopped.")
    # # # joystick_thread.start()
    # # thread1.start()

    # try:
    #     while True:
    #         print("here we are on main loop")
    #         time.sleep(1)
    # except KeyboardInterrupt:
    #     print("KeyboardInterrupt detected in main loop, stopping threads...")
    #     stop_event.stop()
    #     # joystickEventLoop.stop()
    #     # joystick_thread.join()
    #     # thread1.join()
    #     print("Both threads have finished.")
