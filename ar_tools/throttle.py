from time import perf_counter, sleep

class Throttle:
    def __init__(self, hz):
        self._period = 1 / hz
        self._last_proc = perf_counter()
        
    def wait(self):
        elapsed_ = perf_counter() - self._last_proc
        if elapsed_ < self._period:
            sleep(self._period - elapsed_)
        self._last_proc = perf_counter()
            
    def poll(self):
        now_ = perf_counter()
        if now_ - self._last_proc < self._period:
            return False
        else:
            self._last_proc = now_
            return True
            
if __name__ == '__main__':
    throttle_ = Throttle(10)
    while True:
        sleep(0.05)
        throttle_.wait()
        #if throttle_.poll():
        print(perf_counter())
