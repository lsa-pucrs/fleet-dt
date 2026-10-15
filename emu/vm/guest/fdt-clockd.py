#!/usr/bin/env python3
"""vsock clock server of an emulated host (SPEC_v2 Section 7, Clock).

Listens on AF_VSOCK port 5000; for every byte received on a connection it answers with the
CLOCK_REALTIME of the VM in ns (8 bytes, little endian). emu/clockprobe/probe.py reads it.
"""
import socket
import struct
import threading
import time

PORT = 5000


def serve(c):
    with c:
        while True:
            b = c.recv(1)
            if not b:
                return
            c.sendall(struct.pack("<q", time.clock_gettime_ns(time.CLOCK_REALTIME)))


def main():
    s = socket.socket(socket.AF_VSOCK, socket.SOCK_STREAM)
    s.bind((socket.VMADDR_CID_ANY, PORT))
    s.listen(4)
    while True:
        c, _ = s.accept()
        threading.Thread(target=serve, args=(c,), daemon=True).start()


if __name__ == "__main__":
    main()
