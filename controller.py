'''This is an example implementation of the controller component of our emulation system.
    It is used for testing our proof-of-concept implementation and provides basic and hardcoded functionality.
'''

import os
import sys
import socket
import time
from datetime import datetime, timedelta
import struct

# MAC-addresses of used ESP devices, to distinguish them throughout wifi communication.
mactoesp = {
    "24:0A:C4:E8:16:E8" : 1,
    "24:0A:C4:E8:0D:20" : 2,
    "FC:F5:C4:3C:5F:04" : 3,
    "24:0A:C4:E8:0D:A8" : 4,
    "24:0A:C4:E8:10:74" : 5,
    "24:0A:C4:E8:16:BC" : 6,
    "24:0A:C4:E8:0F:F8" : 7,
    "24:0A:C4:E8:0E:E4" : 8,
    "24:0A:C4:E8:15:E8" : 9,
    "24:0A:C4:E8:0D:C0" : 10,
    "84:0D:8E:E3:B4:D0" : 11,
    "84:0D:8E:E3:B5:E0" : 12,
    "84:0D:8E:E3:AB:24" : 13,
    "84:0D:8E:E3:A8:68" : 14,
    "84:0D:8E:E3:A8:48" : 15,
    "84:0D:8E:E3:B4:94" : 16,
    "84:0D:8E:E3:A7:E8" : 17,
    "84:0D:8E:E3:B6:B8" : 18,
    "84:0D:8E:E3:B4:A4" : 19,
    "84:0D:8E:E3:AB:B8" : 20,
    "24:D7:EB:BB:8E:80" : 21,
    "8C:4B:14:15:A0:2C" : 22
}

# Configuration for testbed setup, which are master and which slave devices
esp_slave_macs = [1, 3, 5, 7, 9, 11, 12, 13, 14, 15, 21]
esp_master_macs = [2, 4, 6, 8, 10, 16, 17, 18, 19, 20, 22]

# Helper function to generate array of timestamps for an update-frame.
def generate_times(start, stride, num):
    times = []
    for i in range(0, num):
        times.append(start + i * stride)
    return times

# Helper function to generate array of temperatures for an update-frame.
def generate_temperatures(start, stride, num):
    temps = []
    for i in range(0, num):
        temps.append(start + i * stride)
    return temps

# Helper function to generate array of pressures for an update-frame.
def generate_pressures(start, stride, num):
    presss = []
    for i in range(0, num):
        presss.append(start + i * stride)
    return presss

# Helper function to generate an update frame for BMP180 emulator in specific format.
def generate_update(addr, temperatures, temp_times, pressures, press_times, file=None):
    if file:
        for i in range(0, len(temperatures)):
            file.write(datetime.fromtimestamp(temp_times[i]).strftime("%H:%M:%S").encode("ASCII") + b"A" + hex(addr).encode("ASCII") + b"T" + str(temperatures[i]).encode("ASCII") + b"\n")

        for i in range(0, len(pressures)):
            file.write(datetime.fromtimestamp(press_times[i]).strftime("%H:%M:%S").encode("ASCII") + b"A" + hex(addr).encode("ASCII") + b"P" + str(pressures[i]).encode("ASCII") + b"\n")

    message = addr.to_bytes(1, "little")
    if temperatures:
        message += len(temperatures).to_bytes(2, "little")
        for temp in temperatures:
            message += bytearray(struct.pack("d", temp))

        for time in temp_times:
            message += time.to_bytes(4, "little")
    else:
        message += b"\x00\x00"

    if pressures:
        message += len(pressures).to_bytes(2, "little")
        for press in pressures:
            message += press.to_bytes(8, "little")

        for time in press_times:
            message += time.to_bytes(4, "little")
    else:
        message += b"\x00\x00"

    return message

# Sets the number of updates contained in our update frame
def set_num_updates(num, msg):
    newmsg = num.to_bytes(1, "little") + msg
    return newmsg

# Robust socket sending implementation. Sends message until len(msg) bytes have been transferred.
def robust_send(sock : socket, msg):
    size = len(msg)
    sent = 0
    while size - sent > 0:
        sent += sock.send(msg[sent:])
    return

# Receive until 'sequence' occurs in message from client.
def recv_until(sock : socket, sequence, seconds = -1):
    start = time.time()
    response = b""
    while not response.endswith(sequence):
        response += sock.recv(1)
        if seconds > 0 and int(time.time() - start) >= seconds:
            return response, False
    return response, True

# Send 'ACK' acknowledgement to client ESP.
def ack(sock : socket):
    return recv_until(sock, b"ACK")

# Kill all active processes on controller (uses a process for every ESP to handle communication simultaneously)
def purge_processes(processes):
    for proc in processes:
        proc.kill()

# Get MAC-address of a connected ESP32 client
def get_mac(sock : socket):
    macaddr = 0
    success = False

    while macaddr == 0:
        while not success:
            robust_send(sock, b"MAC")
            response, success  = recv_until(sock, b"ACK")
            macaddr = response.split(b"ACK")[0]
    return macaddr.decode("ASCII")

# Sends an update frame to an ESP32 emulator client
def send_update(sock : socket, update):
    robust_send(sock, b"DATA")
    ack(sock)
    robust_send(sock, len(update).to_bytes(2, "little"))
    ack(sock)
    robust_send(sock, update)
    ack(sock)

# Enables/Disables I2C slave address to querying on ESP32 master client
def config_master_i2c(sock : socket, address, status):
    robust_send(sock, b"MCONFIG")
    ack(sock)
    if status == True:
        robust_send(sock, address.to_bytes(1, "little"))
    else:
        robust_send(sock, (address | 0x80).to_bytes(1, "little"))
    ack(sock)

# Enables/Disables emulator device slave interface on ESP32 slave client
def config_slave_emulator(sock : socket, address, status):
    robust_send(sock, b"SCONFIG")
    ack(sock)
    config = b"\x00\x00"
    if status == True:
        config += address.to_bytes(1, "little")
    else:
        config += (address | 0x80).to_bytes(1, "little")
    robust_send(sock, config)
    ack(sock)

def sync_esp(sock : socket):
    robust_send(sock, b"SYNC")
    response, _ = ack(sock)
    now = datetime.now()
    lower_tolerance = (now - timedelta(seconds=1)).time()
    upper_tolerance = (now + timedelta(seconds=1)).time()
    now = now.time()
    synctime = response.split(b"ACK")[0].decode("ASCII")
    synctime = datetime.strptime(synctime, "%H:%M:%S.%f").time()
    # print(synctime, now)
    if synctime >= lower_tolerance and synctime <= upper_tolerance:
        return True
    else:
        return False
    
# Logs start time of experiment run on ESP32 client    
def experiment_start(sock : socket):
    robust_send(sock, b"START")
    ack(sock)
    
# Logs stop time of experiment run on ESP32 client
def experiment_stop(sock : socket):
    robust_send(sock, b"STOP")
    ack(sock)

# Synchronizes ESP32 client time
def synchronize(sock : socket):
    while not sync_esp(sock):
        pass

# Forks a process for every connected ESP32 client
def fork_clients(sockets, function):
    client_procs = []
    for sock in sockets:
        # Fork child porcess for every connection to handle them simultaneously
        pid = os.fork()
        if pid == 0:
            function(sock)
            sys.exit()
        else:
            client_procs.append(pid)
    
    for pid in client_procs:
        os.waitpid(pid, 0)

# Starts experiment controller
def start_controller(exp_start, exp_end, exp_run):
    global file
    global exp_start_time
    global exp_stop_time
    global nodes

    file = open("results/Controller", "ab+", 0)
    # Create listening socket object
    listen = socket.socket()		

    # All ESPs have to connect to this port (can be other)
    port = 3333			

    # Bind to all IP-interfaces on host-machine using above specified port and allow reuse of address
    listen.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listen.bind(('', port))

    # Listen to incoming connections and queue up to 16 connections while handling a new one
    listen.listen(16)

    # Handle incoming requests
    client_socks = []
    while len(client_socks) < nodes:
        # Establish connection with client
        client, _ = listen.accept()	
        client_socks.append(client)

    #  Synchronize clients system times
    fork_clients(client_socks, synchronize)

    # Set Experiment Start and End times
    exp_start_time = datetime.now() + timedelta(seconds=exp_start)
    exp_stop_time = exp_start_time + timedelta(seconds=exp_end)
    file.write(exp_start_time.strftime("%H:%M:%S.%f")[:-3].encode("ASCII") + b"START\n")
    file.write(exp_stop_time.strftime("%H:%M:%S.%f")[:-3].encode("ASCII") + b"STOP\n")

    #  Activate emulation and measuring interfaces on clients
    fork_clients(client_socks, exp_run)

    while True:
        pass

    return

# Helper to generate extensive 20 min duration emulation curve update for BMP180 emulator
def complete_update(emulator, start_time):
    # Generate Update frame for Emulator
    update = generate_update(emulator, generate_temperatures(-40.0, 0.1, 1201), generate_times(int(datetime.timestamp(start_time)) + 5, 1, 1201), generate_pressures(30000, 67, 1201), generate_times(int(datetime.timestamp(start_time)) + 5, 1, 1201), None)
    return set_num_updates(1, update)

# Generate complete update but split it into four update frames that can be sent out seperately
def complete_update_split_4(emulator, start_time):
    # Generate Update frames for Emulator and put them into list
    update_list = []

    update = generate_update(emulator, generate_temperatures(-40.0, 0.1, 300), generate_times(int(datetime.timestamp(start_time)) + 5, 1, 300), generate_pressures(30000, 67, 300), generate_times(int(datetime.timestamp(start_time)) + 5, 1, 300), None)
    update_list.append(set_num_updates(1, update))

    update = generate_update(emulator, generate_temperatures(-10.0, 0.1, 300), generate_times(int(datetime.timestamp(start_time)) + 305, 1, 300), generate_pressures(50100, 67, 300), generate_times(int(datetime.timestamp(start_time)) + 305, 1, 300), None)
    update_list.append(set_num_updates(1, update))

    update = generate_update(emulator, generate_temperatures(20.0, 0.1, 300), generate_times(int(datetime.timestamp(start_time)) + 605, 1, 300), generate_pressures(70200, 67, 300), generate_times(int(datetime.timestamp(start_time)) + 605, 1, 300), None)
    update_list.append(set_num_updates(1, update))

    update = generate_update(emulator, generate_temperatures(50.0, 0.1, 301), generate_times(int(datetime.timestamp(start_time)) + 905, 1, 301), generate_pressures(90300, 67, 301), generate_times(int(datetime.timestamp(start_time)) + 905, 1, 301), None)
    update_list.append(set_num_updates(1, update))

    return update_list

# Generate shorter, 7.5 minute, update frame
def seven_five_minutes(emulator, start_time):
    # Generate Update frame for Emulator
    update = generate_update(emulator, generate_temperatures(-2.5, 0.1, 451), generate_times(int(datetime.timestamp(start_time)) + 5, 1, 451), generate_pressures(40000, 67, 451), generate_times(int(datetime.timestamp(start_time)) + 5, 1, 451), None)
    return set_num_updates(1, update)

# Generate 5 minute update frame
def five_minutes(emulator, start_time):
    # Generate Update frame for Emulator
    update = generate_update(emulator, generate_temperatures(5.0, 0.1, 301), generate_times(int(datetime.timestamp(start_time)) + 5, 1, 301), generate_pressures(50000, 67, 301), generate_times(int(datetime.timestamp(start_time)) + 5, 1, 301), None)
    return set_num_updates(1, update)

# Configure and runESP32 I2C masters only (used during experiment where multiple masters query multiple genuine BMP180s)
def master_only(sock : socket):
    global exp_start_time
    global exp_stop_time
    global emulators_per_slave

    for i in range(0, emulators_per_slave):
        config_master_i2c(sock, i, True)

    while datetime.now() < exp_start_time:
        time.sleep(0.05)

    experiment_start(sock)

    while datetime.now() < exp_stop_time:
        time.sleep(0.05)

    experiment_stop(sock)

    for i in range(0, emulators_per_slave):
        config_master_i2c(sock, i, False)

# Configure and run I2C masters and hardware based slaves for experiment run (one update is sent in experiment)
def masters_and_hardwareslaves_one_update(sock : socket):
    global file
    global exp_start_time
    global exp_stop_time
    global emulators_per_slave

    # Get MAC
    mac = get_mac(sock)

    # Check corresponding ESP
    esp = mactoesp[mac]
    if esp in esp_slave_macs:
        for i in range(0, emulators_per_slave):
            # Generate and Send Update Frame for Emulator
            send_update(sock, complete_update(i, exp_start_time))

    while datetime.now() < exp_start_time:
        time.sleep(0.05)

    experiment_start(sock)

    while datetime.now() < exp_stop_time:
        time.sleep(0.05)

    experiment_stop(sock)

# Configure and run I2C masters and hardware based slaves experiment (four updates are sent throughout experiment)
def masters_and_hardwareslaves_four_updates(sock : socket):
    global file
    global exp_start_time
    global exp_stop_time
    global emulators_per_slave

    # Get MAC
    mac = get_mac(sock)

    # Check corresponding ESP
    esp = mactoesp[mac]
    list_of_updates = []
    if esp in esp_slave_macs:
        for i in range(0, emulators_per_slave):
            # Generate and Send Update Frame for Emulator
            list_of_updates.append(complete_update_split_4(i, exp_start_time))
            # Send initial update frame one
            send_update(sock, list_of_updates[i][0])

    while datetime.now() < exp_start_time:
        time.sleep(0.05)

    experiment_start(sock)

    while datetime.now() < exp_start_time + timedelta(seconds=155):
        time.sleep(0.05)

    if esp in esp_slave_macs:
        for i in range(0, emulators_per_slave):
            # Send update frame two
            send_update(sock, list_of_updates[i][1])

    while datetime.now() < exp_start_time + timedelta(seconds=455):
        time.sleep(0.05)

    if esp in esp_slave_macs:
        for i in range(0, emulators_per_slave):
            # Send update frame three
            send_update(sock, list_of_updates[i][2])

    while datetime.now() < exp_start_time + timedelta(seconds=755):
        time.sleep(0.05)

    if esp in esp_slave_macs:
        for i in range(0, emulators_per_slave):
            # Send update frame four
            send_update(sock, list_of_updates[i][3])

    while datetime.now() < exp_stop_time:
        time.sleep(0.05)

    experiment_stop(sock)

# Configure and run I2C masters and software slaves (one update)
def masters_and_bitbangslaves_one_update(sock : socket):
    global file
    global exp_start_time
    global exp_stop_time
    global emulators_per_slave

    # Get MAC
    mac = get_mac(sock)

    # Check corresponding ESP
    esp = mactoesp[mac]
    if esp in esp_slave_macs:
        for i in range(0, emulators_per_slave):
            config_slave_emulator(sock,i, True)

            # Generate and Send Update Frame for Emulator
            send_update(sock, complete_update(i, exp_start_time))

    while datetime.now() < exp_start_time:
        time.sleep(0.05)

    experiment_start(sock)

    while datetime.now() < exp_stop_time:
        time.sleep(0.05)

    experiment_stop(sock)

    if esp in esp_slave_macs:
        for i in range(0, emulators_per_slave):
            config_slave_emulator(sock, i, False)
            pass

# Configure and run I2C masters and software slaves (four updates)
def masters_and_bitbangslaves_four_updates(sock : socket):
    global file
    global exp_start_time
    global exp_stop_time
    global emulators_per_slave

    # Get MAC
    mac = get_mac(sock)

    # Check corresponding ESP
    esp = mactoesp[mac]
    list_of_updates = []
    if esp in esp_slave_macs:
        for i in range(0, emulators_per_slave):
            config_slave_emulator(sock,i, True)
            # Generate and Send Update Frame for Emulator
            list_of_updates.append(complete_update_split_4(i, exp_start_time))
            # Send initial update frame one
            send_update(sock, list_of_updates[i][0])

    while datetime.now() < exp_start_time:
        time.sleep(0.05)

    experiment_start(sock)

    while datetime.now() < exp_start_time + timedelta(seconds=155):
        time.sleep(0.05)

    if esp in esp_slave_macs:
        for i in range(0, emulators_per_slave):
            # Send update frame two
            send_update(sock, list_of_updates[i][1])

    while datetime.now() < exp_start_time + timedelta(seconds=455):
        time.sleep(0.05)

    if esp in esp_slave_macs:
        for i in range(0, emulators_per_slave):
            # Send update frame three
            send_update(sock, list_of_updates[i][2])

    while datetime.now() < exp_start_time + timedelta(seconds=755):
        time.sleep(0.05)

    if esp in esp_slave_macs:
        for i in range(0, emulators_per_slave):
            # Send update frame four
            send_update(sock, list_of_updates[i][3])

    while datetime.now() < exp_stop_time:
        time.sleep(0.05)

    experiment_stop(sock)

    if esp in esp_slave_macs:
        for i in range(0, emulators_per_slave):
            config_slave_emulator(sock, i, False)
            pass

# Start and stop times for experiment set here
def start_and_stop(start_time, stop_time):
    global file
    global exp_start_time
    global exp_stop_time

    # Set Experiment Start and End times
    exp_start_time = datetime.strptime(start_time, "%Y-%m-%d %H:%M:%S")
    exp_stop_time = datetime.strptime(stop_time, "%Y-%m-%d %H:%M:%S")
    file.write(exp_start_time.strftime("%H:%M:%S.%f")[:-3].encode("ASCII") + b"START\n")
    file.write(exp_stop_time.strftime("%H:%M:%S.%f")[:-3].encode("ASCII") + b"STOP\n")

# Main kicks off experiment run
if __name__ == "__main__":
    global nodes
    global masters
    global slaves
    global emulators_per_slave

    # Create and or Empty Log-File for Controller
    open("results/Controller", "wb+", 0).close()

    # Configure the experiment
    masters = 5
    slaves = 5
    nodes = masters + slaves
    emulators_per_slave = 128
    start_controller(30, 1230, masters_and_bitbangslaves_four_updates)
    pass