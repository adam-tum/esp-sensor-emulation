'''
This module is used to build and flash multiple ESP32s with the same code simultaneously.

This is makes an experiment setup easier for our proof-of-concept test runs, but is not necessary.

Makes use of idf.py tool and can't run independently.
'''

import os
import sys
import subprocess
import multiprocessing

flash_procs = []

# Flashes the ESPs.
def esp_port(esp, lock):
    flashed = False
    with open("results/ESP" + str(esp), "wb+", 0) as file:
        command = 'idf.py flash monitor -p /dev/ttyESP' + str(esp)
        proc = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)

        while True:
            line = proc.stdout.readline()
            if b"Starting Main-Application." in line and (not flashed):
                flashed = True
            if line and flashed:
                file.write(line)

# Start multiple processes to flash ESPs simultaneously.
def flash_esps(ports, lock):
    for esp in ports:
        proc = multiprocessing.Process(target=esp_port, args=(esp,lock))
        proc.start()
        flash_procs.append(proc)
        
# Kill all flasher processes.
def purge_processes(processes):
    for proc in processes:
        proc.kill()

'''
ESPs to flash are all connected via USB-hub.

All connected ESPs have been symlinked in /dev/tty to ESP*, where the asterisk is a unique number for each one.
Therefore, simply calling idf.py build and then idf.py flash monitor -p /dev/ttyESP* for each connected ESP number does the trick.

Console arguments based usage.
Calling "python3 flasher.py 0 3" should flash ESP0, ESP1, ESP2 and ESP3. Calling "python flasher.py [0,1,2,3]" does the same.
'''
if __name__ == "__main__":
    lock = multiprocessing.Lock()
    os.system("idf.py build")

    esps = []
    if "[" in sys.argv[1] and "]" in sys.argv[1]:
        # Parse List of ESPs to flash.
        esps = [int(esp) for esp in sys.argv[1][1:-1].split(",")]
        flash_esps(esps, lock)
    else:
        flash_esps(range(int(sys.argv[1]), int(sys.argv[2]) + 1), lock)