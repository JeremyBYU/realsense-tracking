from multiprocessing import Pool
import os
def f(x):
    import time 
    print(f"Doing work for {x} on pid: {os.getpid()}")
    time.sleep(100)
    return x*x

if __name__ == '__main__':
    with Pool(5) as p:
        print(p.map(f, [1, 2, 3, 4, 5]))