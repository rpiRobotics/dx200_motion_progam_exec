import socket
import sys
import time
import binascii


UDP_IP = "192.168.20.31"
UDP_PORT = 10040

Filenamecommand=b"YERC \00"+b"\x24"+b"\00\03\01\00\00\00\00\00\0099999999"+b"\x87"+b"\00\01\00\00\02\00\00"+b"WAIT1"+b"\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00"

Runcommand=b"YERC \00\04\00\03\01\00\00\00\00\00\0099999999"+b"\x86"+b"\00\01\00\01"+b"\x10"+b"\00\00\01\00\00\00"
hello=binascii.hexlify(bytearray(Filenamecommand))
status_check=b"YERC \00\00\00\03\01\00\00\00\00\00\0099999999"+b"\x72"+b"\00\01\00\00\01\00\00"

#print(status_check)
hello=binascii.hexlify(bytearray(status_check))
print(hello)
MESSAGE=Filenamecommand



print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % status_check)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
data, addr = sock.recvfrom(1024)
time.sleep(1)
MESSAGE=Runcommand
print("message: %s" % MESSAGE)
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
data, addr = sock.recvfrom(1024)
MESSAGE=status_check
time.sleep(0.5)
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    bytes=bytearray(data)
    
    hello=binascii.hexlify(bytearray(data))
    
    print("received message: %s" % hello)
    if(hello[-15]=="a"):
        #Bit 3 is the bit encoding the status of the program, but bit 1 is normally true so "a" in this position means the program is running
        sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
    else:
        break
    time.sleep(0.1)
    #print(byte)
