"""
log all socket traffic to/from an ethernet Prologix device
"""

import select
import socket
import time


def proxy(ipaddr: str, port: int = 1234):
    """
    this proxy forwards all traffic to/from an ethernet Prologix
    adapter so that it can be logged to stdout
    """
    # set up connection to Prologix adapter
    prologix = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    prologix.connect((ipaddr, port))
    prologix.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    # set up server to listen for connections
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("", 1234))
    server.listen()
    epoch = time.time()

    # use select to wait for a connection
    rd, _wr, _ex = select.select([server], [], [])
    conn, addr = server.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f"{time.time() - epoch:7.3f} connection from {addr=}")

    while True:
        rd, _wr, _ex = select.select([conn, prologix], [], [])
        if conn in rd:
            data = conn.recv(1024)
            if len(data) == 0:
                break

            prologix.sendall(data)
            print(f"{time.time() - epoch:7.3f} --> {data=}")
        if prologix in rd:
            data = prologix.recv(1024)
            if len(data) == 0:
                break

            conn.sendall(data)
            print(f"{time.time() - epoch:7.3f} <-- {data=}")

    # close all the sockets and exit
    server.close()
    prologix.close()
    conn.close()
    print(f"{time.time() - epoch:7.3f} connection closed")
