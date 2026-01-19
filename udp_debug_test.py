import socket
import struct
import time

UDP_IP = "138.38.226.46"  # update if needed
UDP_PORT = 25000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(2.0)

def send_test(distance=0.0, angle=0.0):
    packet = struct.pack('<ff', float(distance), float(angle))
    try:
        sent = sock.sendto(packet, (UDP_IP, UDP_PORT))
        print(f"sent {sent} bytes to {UDP_IP}:{UDP_PORT}")
    except Exception as e:
        print("send error:", e)
        return

    # Listen briefly for any reply (many UDP listeners won't reply)
    try:
        data, addr = sock.recvfrom(1024)
        print("received reply from", addr, "len=", len(data))
    except socket.timeout:
        print("no reply (timeout)")

if __name__ == '__main__':
    print("UDP debug test to", UDP_IP, UDP_PORT)
    send_test(0.0, 0.0)
    time.sleep(0.1)
    send_test(1.23, 45.0)
    sock.close()
