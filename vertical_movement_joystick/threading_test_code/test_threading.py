import threading
import time

class StoppableThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

def thread_function_1(stop_event):
    # Function for the first thread
    try:
        while not stop_event.stopped():
            print("Thread 1 executing...")
            time.sleep(1)
    except KeyboardInterrupt:
        print("Thread 1 interrupted by keyboard.")

def thread_function_2(stop_event):
    # Function for the second thread
    try:
        while not stop_event.stopped():
            print("Thread 2 executing...")
            time.sleep(1)
    except KeyboardInterrupt:
        print("Thread 2 interrupted by keyboard.")

def main():
    # Creating stop event
    stop_event = StoppableThread()

    # Creating threads
    thread1 = threading.Thread(target=thread_function_1, args=(stop_event,))
    thread2 = threading.Thread(target=thread_function_2, args=(stop_event,))

    # Starting threads
    thread1.start()
    thread2.start()

    try:
        while True:
            print("Main Program is running")
            time.sleep(1)
    except KeyboardInterrupt:
        print("KeyboardInterrupt detected, stopping threads...")
        stop_event.stop()
        thread1.join()
        thread2.join()

        print("Both threads have finished.")

if __name__ == "__main__":
    main()

