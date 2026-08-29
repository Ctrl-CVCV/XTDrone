#!/usr/bin/env python3
"""Dump raw hex of MAVLink datagrams for a given dst port. Run as root.

Usage: python3 sniff_hex.py <dst-port> <seconds> <outfile>
"""
import socket
import struct
import sys
import time

port = int(sys.argv[1])
seconds = float(sys.argv[2])
out = sys.argv[3]

s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0800))
s.bind(("lo", 0))
s.settimeout(1.0)

seen = 0
deadline = time.monotonic() + seconds
with open(out, "w") as f:
    while time.monotonic() < deadline:
        try:
            frame = s.recv(65535)
        except socket.timeout:
            continue
        if len(frame) < 14:
            continue
        if struct.unpack_from(">H", frame, 12)[0] != 0x0800:
            continue
        ip = frame[14:]
        if len(ip) < 20 or (ip[0] >> 4) != 4:
            continue
        ihl = (ip[0] & 0x0F) * 4
        if ip[9] != 17:
            continue
        udp = ip[ihl:]
        if len(udp) < 8:
            continue
        sport, dport, ulen = struct.unpack_from(">HHH", udp, 0)
        if dport != port:
            continue
        payload = udp[8:ulen]
        if not payload:
            continue
        # find mavlink sync in this datagram
        for i, b in enumerate(payload):
            if b in (0xFD, 0xFE) and len(payload) - i >= 12:
                plen = payload[i + 1]
                if i + 10 + plen + 2 <= len(payload):
                    msgid = payload[i + 5]
                    if b == 0xFD:
                        msgid = payload[i + 7] | (payload[i + 8] << 8) | (payload[i + 9] << 16)
                    if msgid == 84 or msgid == 0:
                        hexstr = payload[i:i + 10 + plen + 2].hex()
                        f.write("%d %d %s %s\n" % (sport, dport, msgid, hexstr))
                        seen += 1
                    break
        if seen >= 8:
            break
print("wrote %d messages" % seen)
