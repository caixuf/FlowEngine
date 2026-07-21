#!/usr/bin/env python3
"""
Foxglove WebSocket Bridge — 实时 3D 可视化桥接

FlowEngine → JSON state file → 本脚本 → Foxglove Studio (ws://localhost:8765)

用法:
  python3 tools/foxglove_bridge.py [--port 8765] [--json-file /tmp/flow_topology.json]

Foxglove Studio 侧:
  Open Connection → Foxglove WebSocket → ws://localhost:8765
"""

import asyncio
import json
import struct
import hashlib
import base64
import os
import sys
import time
import argparse
from pathlib import Path

# ── WebSocket framing (minimal, no external deps) ──────────

def ws_handshake(headers):
    """Extract Sec-WebSocket-Key and build accept response."""
    key = None
    for line in headers.split('\r\n'):
        if line.lower().startswith('sec-websocket-key:'):
            key = line.split(':',1)[1].strip()
    if not key: return None
    accept = base64.b64encode(
        hashlib.sha1((key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()
    ).decode()
    return (
        'HTTP/1.1 101 Switching Protocols\r\n'
        'Upgrade: websocket\r\n'
        'Connection: Upgrade\r\n'
        f'Sec-WebSocket-Accept: {accept}\r\n'
        'Sec-WebSocket-Protocol: foxglove.websocket.v1\r\n\r\n'
    ).encode()

def ws_send_text(writer, payload):
    """Send a text frame (opcode 0x1)."""
    data = payload.encode()
    frame = bytearray()
    frame.append(0x81)  # FIN + text opcode
    length = len(data)
    if length < 126:
        frame.append(length)
    elif length < 65536:
        frame.append(126)
        frame.extend(struct.pack('>H', length))
    else:
        frame.append(127)
        frame.extend(struct.pack('>Q', length))
    frame.extend(data)
    writer.write(bytes(frame))

def ws_send_binary(writer, data):
    """Send a binary frame (opcode 0x2)."""
    frame = bytearray()
    frame.append(0x82)  # FIN + binary opcode
    length = len(data)
    if length < 126:
        frame.append(length)
    elif length < 65536:
        frame.append(126)
        frame.extend(struct.pack('>H', length))
    else:
        frame.append(127)
        frame.extend(struct.pack('>Q', length))
    frame.extend(data)
    writer.write(bytes(frame))

def ws_read_frame(reader):
    """Read one WebSocket frame, return (opcode, payload)."""
    try:
        hdr = yield from reader.readexactly(2)
    except:
        return None, None
    opcode = hdr[0] & 0x0F
    length = hdr[1] & 0x7F
    if length == 126:
        length = struct.unpack('>H', (yield from reader.readexactly(2)))[0]
    elif length == 127:
        length = struct.unpack('>Q', (yield from reader.readexactly(8)))[0]
    mask = (yield from reader.readexactly(4)) if (hdr[1] & 0x80) else None
    payload = yield from reader.readexactly(length)
    if mask:
        payload = bytes(b ^ mask[i%4] for i, b in enumerate(payload))
    return opcode, payload

# ── Foxglove protocol messages ─────────────────────────────

def foxglove_server_info():
    return json.dumps({"op":"serverInfo","name":"FlowEngine",
        "capabilities":["clientPublish","time"],"supportedEncodings":["json"]})

def foxglove_advertise(ch):
    return json.dumps({"op":"advertise","channels":[{
        "id":ch["id"],"topic":ch["topic"],"encoding":"json",
        "schemaName":ch.get("schemaName",""),"schema":ch.get("schema","{}")}]})

def foxglove_message_data(ch_id, seq, payload_bytes):
    """Binary: 1B op(1) + 4B channel_id(LE) + 4B seq(LE) + payload"""
    hdr = struct.pack('<BI', 1, ch_id) + struct.pack('<I', seq)
    return hdr + payload_bytes

def foxglove_time(ts_ns):
    return struct.pack('<BQ', 2, ts_ns)  # op=2 + 8B timestamp

# ── Data source ────────────────────────────────────────────

class StateWatcher:
    def __init__(self, json_path):
        self.path = json_path
        self.last_mtime = 0
        self.topics = {}   # topic → {pub, del, freq, ...}
        self.vehicle = {}
        self.channels = [] # list of {"id","topic","schemaName","schema"}

    def poll(self):
        try:
            if not os.path.exists(self.path): return False
            mtime = os.path.getmtime(self.path)
            if mtime == self.last_mtime: return False
            self.last_mtime = mtime
            with open(self.path) as f:
                data = json.load(f)
            # Extract topics from metrics
            metrics = data.get("metrics", {})
            ts = metrics.get("topics", [])
            self.topics = {}
            for t in ts:
                self.topics[t.get("topic","")] = t
            self.vehicle = metrics.get("vehicle", {})
            return True
        except:
            return False

    def ensure_channels(self):
        """Build channel list from topic names."""
        existing = {ch["topic"] for ch in self.channels}
        changed = False
        for topic in sorted(self.topics.keys()):
            if topic not in existing:
                ch_id = len(self.channels) + 1
                self.channels.append({
                    "id": ch_id, "topic": topic,
                    "schemaName": topic.split("/")[-1] if "/" in topic else topic,
                    "schema": json.dumps({"type":"object","properties":{
                        "pub":{"type":"integer"},"del":{"type":"integer"},
                        "drop":{"type":"integer"},"lat_us":{"type":"integer"},
                        "freq":{"type":"number"},"subs":{"type":"integer"}
                    }})
                })
                changed = True
        # Always add vehicle state channel
        if "vehicle/state" not in existing:
            ch_id = len(self.channels) + 1
            self.channels.append({
                "id": ch_id, "topic": "/vehicle/state",
                "schemaName": "VehicleState",
                "schema": json.dumps({"type":"object","properties":{
                    "speed":{"type":"number"},"target_speed":{"type":"number"},
                    "throttle":{"type":"number"},"brake":{"type":"number"},
                    "x":{"type":"number"},"y":{"type":"number"},
                    "error":{"type":"number"}
                }})
            })
            changed = True
        return changed

    def get_message_data(self):
        """Generate message payloads for each topic + vehicle state."""
        msgs = []
        for ch in self.channels:
            if ch["topic"].startswith("/vehicle"):
                # Vehicle state is a special channel
                if self.vehicle:
                    msgs.append((ch["id"], json.dumps(self.vehicle).encode()))
                continue
            t = self.topics.get(ch["topic"])
            if t:
                msgs.append((ch["id"], json.dumps(t).encode()))
        return msgs

# ── Main server ────────────────────────────────────────────

async def handle_client(reader, writer, watcher):
    """Handle one Foxglove WebSocket client."""
    addr = writer.get_extra_info('peername')
    print(f"[foxglove-bridge] client connected: {addr}")

    # Read HTTP upgrade request
    request = b''
    while b'\r\n\r\n' not in request:
        chunk = await reader.read(4096)
        if not chunk: break
        request += chunk
    headers = request.decode(errors='replace')

    # WebSocket handshake
    response = ws_handshake(headers)
    if not response:
        writer.close(); return
    writer.write(response)
    await writer.drain()

    # Send server info
    ws_send_text(writer, foxglove_server_info())
    await writer.drain()

    # Send channel advertisements
    watcher.ensure_channels()
    for ch in watcher.channels:
        ws_send_text(writer, foxglove_advertise(ch))
    await writer.drain()

    # Send time
    ws_send_binary(writer, foxglove_time(int(time.time() * 1e9)))
    await writer.drain()

    # Subscribed channels (track client subscriptions)
    subscribed = set()

    # Main loop: poll data + handle client messages
    seq = 0
    channels_advertised = False
    while True:
        try:
            # Non-blocking read for client frames
            try:
                op, payload = await asyncio.wait_for(
                    asyncio.ensure_future(read_frame(reader)), timeout=0.1)
                if op == 0x1:  # text
                    msg = json.loads(payload.decode())
                    if msg.get("op") == "subscribe":
                        for ch in msg.get("subscriptions", []):
                            subscribed.add(ch.get("channelId", 0))
                    elif msg.get("op") == "unsubscribe":
                        for ch in msg.get("subscriptions", []):
                            subscribed.discard(ch.get("channelId", 0))
                elif op == 0x8:  # close
                    break
            except (asyncio.TimeoutError, TimeoutError):
                pass

            # Poll state file
            if watcher.poll():
                # Only advertise channels when they actually change
                if not channels_advertised or watcher.ensure_channels():
                    channels_advertised = True
                    for ch in watcher.channels:
                        ws_send_text(writer, foxglove_advertise(ch))
                # Send time
                ws_send_binary(writer, foxglove_time(int(time.time() * 1e9)))
                # Send message data for all channels
                for ch_id, payload in watcher.get_message_data():
                    seq += 1
                    data = foxglove_message_data(ch_id, seq, payload)
                    ws_send_binary(writer, data)

                # Drain only when data was actually sent, and don't
                # await it — TCP buffers naturally; blocking drain was
                # a major source of frame stutter in the WebSocket bridge.
                # Use create_task so the write is non-blocking.
                asyncio.ensure_future(writer.drain())

        except (ConnectionResetError, BrokenPipeError, OSError):
            break

    print(f"[foxglove-bridge] client disconnected: {addr}")
    writer.close()

async def read_frame(reader):
    """Async wrapper for ws_read_frame."""
    hdr = await reader.readexactly(2)
    opcode = hdr[0] & 0x0F
    length = hdr[1] & 0x7F
    if length == 126:
        length = struct.unpack('>H', await reader.readexactly(2))[0]
    elif length == 127:
        length = struct.unpack('>Q', await reader.readexactly(8))[0]
    mask = await reader.readexactly(4) if (hdr[1] & 0x80) else None
    payload = await reader.readexactly(length)
    if mask:
        payload = bytes(b ^ mask[i%4] for i, b in enumerate(payload))
    return opcode, payload

async def main(args):
    watcher = StateWatcher(args.json_file)
    print(f"[foxglove-bridge] watching: {args.json_file}")
    print(f"[foxglove-bridge] listening: ws://0.0.0.0:{args.port}")
    print(f"[foxglove-bridge] Foxglove Studio → Open Connection → WebSocket → ws://localhost:{args.port}")

    server = await asyncio.start_server(
        lambda r, w: handle_client(r, w, watcher),
        '0.0.0.0', args.port)
    await server.serve_forever()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Foxglove WebSocket Bridge')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--json-file', type=str,
                        default=os.environ.get('FLOWENGINE_STATE_FILE', '/tmp/flow_topology.json'))
    args = parser.parse_args()
    asyncio.run(main(args))
