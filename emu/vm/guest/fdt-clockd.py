#!/usr/bin/env python3
"""Clock server of an emulated host for the host probe (SPEC_v2 Section 7, Clock).

For every byte received it answers with the CLOCK_REALTIME of the VM in ns (8 bytes, little endian);
emu/clockprobe/probe.py reads it. Channel: the virtio-serial port /dev/virtio-ports/fdt.clock when the VM
has one (hosts without access to /dev/vhost-vsock, emu/run.py clock_channel), else AF_VSOCK port 5000.
"""
import os
import select
import socket
import struct
import threading
import time

PORT = 5000
SERIAL = "/dev/virtio-ports/fdt.clock"


def now():
    return struct.pack("<q", time.clock_gettime_ns(time.CLOCK_REALTIME))


def serve(c):
    with c:
        while True:
            b = c.recv(1)
            if not b:
                return
            c.sendall(now())


def serve_serial(path):
    """virtio-serial: the port reports POLLHUP while no host program has the chardev open"""
    fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
    p = select.poll()
    p.register(fd, select.POLLIN)
    while True:
        ev = p.poll(1000)
        if not ev:
            continue
        if ev[0][1] & select.POLLIN:
            try:
                b = os.read(fd, 64)
            except BlockingIOError:
                continue
            if b:
                os.write(fd, now() * len(b))
                continue
        time.sleep(0.05)    # host side not connected


def main():
    for _ in range(50):
        if os.path.exists(SERIAL):
            return serve_serial(SERIAL)
        time.sleep(0.1)
    s = socket.socket(socket.AF_VSOCK, socket.SOCK_STREAM)
    s.bind((socket.VMADDR_CID_ANY, PORT))
    s.listen(4)
    while True:
        c, _ = s.accept()
        threading.Thread(target=serve, args=(c,), daemon=True).start()


if __name__ == "__main__":
    main()
