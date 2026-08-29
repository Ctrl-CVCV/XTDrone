#!/usr/bin/env python3
"""Sniff loopback MAVLink between MAVROS and PX4 to debug iris_2 OFFBOARD.

Listens on lo for UDP to/from ports 14542/14582 and decodes
SET_POSITION_TARGET_LOCAL_NED (84) and COMMAND_LONG (76).
Run as root: python3 sniff_mavlink.py [seconds] [port-filter]

Usage example:
    python3 -u sniff_mavlink.py 30 14582
"""
import socket
import struct
import sys
import time

PORT_FILTERS = {14540, 14541, 14542, 14580, 14581, 14582}

MSG_SET_POS_TARGET_LOCAL_NED = 84
MSG_COMMAND_LONG = 76
MSG_HEARTBEAT = 0


def decode_mavlink(data):
    """Try to find and decode MAVLink v1/v2 message(s) in data. Returns list of dicts."""
    msgs = []
    i = 0
    n = len(data)
    while i < n - 5:
        if data[i] == 0xFD and i + 9 < n:  # MAVLink v2
            plen = data[i + 1]
            incompat = data[i + 2]
            msgid = data[i + 7] | (data[i + 8] << 8) | (data[i + 9] << 16)
            sysid = data[i + 5]
            compid = data[i + 6]
            start = i + 10
            end = start + plen
            if end + 2 <= n:
                payload = data[start:end]
                msgs.append(_decode(plen, sysid, compid, msgid, payload))
                i = end + 2 + (13 if (incompat & 0x01) else 0)
                continue
        elif data[i] == 0xFE and i + 7 < n:  # MAVLink v1
            plen = data[i + 1]
            sysid = data[i + 3]
            compid = data[i + 4]
            msgid = data[i + 5]
            start = i + 6
            end = start + plen
            if end + 2 <= n:
                payload = data[start:end]
                msgs.append(_decode(plen, sysid, compid, msgid, payload))
                i = end + 2
                continue
        i += 1
    return msgs


def _decode(plen, sysid, compid, msgid, payload):
    info = {"sysid": sysid, "compid": compid, "msgid": msgid}
    if msgid == MSG_SET_POS_TARGET_LOCAL_NED and plen >= 53:
        (tb,) = struct.unpack_from("<I", payload, 0)
        x, y, z = struct.unpack_from("<3f", payload, 4)
        vx, vy, vz = struct.unpack_from("<3f", payload, 16)
        afx, afy, afz = struct.unpack_from("<3f", payload, 28)
        yaw, yaw_rate = struct.unpack_from("<2f", payload, 40)
        (type_mask,) = struct.unpack_from("<H", payload, 48)
        coord_frame, tgt_sys, tgt_comp = struct.unpack_from("<BBB", payload, 50)
        info.update({
            "kind": "SET_POS",
            "t": tb, "x": x, "y": y, "z": z,
            "vx": vx, "vy": vy, "vz": vz,
            "afx": afx, "afy": afy, "afz": afz,
            "yaw": yaw, "yaw_rate": yaw_rate,
            "type_mask": type_mask, "frame": coord_frame,
            "target_sys": tgt_sys, "target_comp": tgt_comp,
        })
    elif msgid == MSG_COMMAND_LONG and plen >= 33:
        (command,) = struct.unpack_from("<H", payload, 0)
        params = struct.unpack_from("<7f", payload, 2)
        conf, tgt_sys, tgt_comp = struct.unpack_from("<BBB", payload, 30)
        info.update({
            "kind": "CMD_LONG", "command": command, "params": params,
            "target_sys": tgt_sys, "target_comp": tgt_comp,
        })
    elif msgid == MSG_HEARTBEAT:
        info["kind"] = "HEARTBEAT"
    else:
        info["kind"] = "MSG_%d" % msgid
    return info


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    port_filter = int(sys.argv[2]) if len(sys.argv) > 2 else None

    s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0800))
    s.bind(("lo", 0))
    s.settimeout(1.0)

    counts = {}
    deadline = time.monotonic() + seconds
    print("sniffing lo for %s s (port filter %s) ..." % (seconds, port_filter))
    while time.monotonic() < deadline:
        try:
            frame = s.recv(65535)
        except socket.timeout:
            continue
        if len(frame) < 14:
            continue
        eth_proto = struct.unpack_from(">H", frame, 12)[0]
        if eth_proto != 0x0800:
            continue
        ip = frame[14:]
        if len(ip) < 20 or (ip[0] >> 4) != 4:
            continue
        ihl = (ip[0] & 0x0F) * 4
        proto = ip[9]
        if proto != 17:  # UDP
            continue
        src = socket.inet_ntoa(ip[12:16])
        dst = socket.inet_ntoa(ip[16:20])
        udp = ip[ihl:]
        if len(udp) < 8:
            continue
        sport, dport, ulen = struct.unpack_from(">HHH", udp, 0)
        payload = udp[8:ulen]
        if port_filter is not None:
            if sport != port_filter and dport != port_filter:
                continue
        elif sport not in PORT_FILTERS and dport not in PORT_FILTERS:
            continue

        for msg in decode_mavlink(payload):
            key = msg["kind"]
            counts[key] = counts.get(key, 0) + 1
            direction = "->" if dport in PORT_FILTERS else "<-"
            detail = ""
            if msg["kind"] == "SET_POS":
                detail = "tsys=%s x=%.3f y=%.3f z=%.3f vx=%.3f vy=%.3f vz=%.3f mask=0x%x frame=%s" % (
                    msg["target_sys"], msg["x"], msg["y"], msg["z"],
                    msg["vx"], msg["vy"], msg["vz"],
                    msg["type_mask"], msg["frame"])
            elif msg["kind"] == "CMD_LONG":
                detail = "cmd=%s tsys=%s p1=%.1f p2=%.1f p3=%.1f" % (
                    msg["command"], msg["target_sys"], msg["params"][0], msg["params"][1], msg["params"][2])
            print("%.1f %s:%d %s %s:%d sys=%d %s %s" % (
                time.monotonic(), src, sport, direction, dst, dport,
                msg["sysid"], msg["kind"], detail))
    print("\n=== totals ===")
    for k, v in sorted(counts.items()):
        print("%-10s %d" % (k, v))


if __name__ == "__main__":
    main()
