"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

VW MQB traffic sign recognition (VZE): the instrument cluster draws the sign from BAP messages the front camera
sends on CAN ID 0x17332110 (BAP LSG 0x21, function 0x10). panda blocks the camera's copies (VZE safety flag);
this relay forwards every camera BAP frame to the car side unchanged, except the sign payload, which is replaced
with openpilot's speed limit. A status change (BAP opcode 4) is injected whenever the limit changes.

BAP over CAN, as seen from VW cameras:
  single frame:  [hdr_hi, hdr_lo, data...]                     byte0 bit7 = 0
  multi frame:   start [0x80 | ch<<4 | len>>8, len & 0xFF, hdr_hi, hdr_lo, data x4]
                 cont. [0xC0 | ch<<4 | idx, data x7]
  hdr = opcode << 12 | lsg << 6 | fct
Sign payload (18 bytes): [type, value, cond, 0, source, 0, <second sign x6>, 0 x6]; type 0x01 = speed limit (km/h),
source 0x10 as sent by an EU A3 camera; all zeros = no sign.
"""
from collections import deque

VZE_BAP_ADDR = 0x17332110
VZE_LSG = 0x21
VZE_FCT_SIGN = 0x10
# Camera status bits. 0x08 makes the cluster show "Traffic sign recognition restricted" (seen on a North American camera
# once navigation data told it the country). The camera's own sign is replaced, so its restriction does not apply.
VZE_FCT_STATUS = 0x13
VZE_STATUS_RESTRICTED = 0x08
OP_HEARTBEAT_STATUS = 3
OP_STATUS = 4
SIGN_PAYLOAD_LEN = 18
MAX_AGE_NS = 200_000_000  # drop camera frames that waited longer than this (e.g. before panda engaged the relay)


def bap_header(op: int, lsg: int, fct: int) -> bytes:
  h = (op << 12) | (lsg << 6) | fct
  return bytes([h >> 8, h & 0xFF])


def parse_header(hi: int, lo: int) -> tuple[int, int, int]:
  h = (hi << 8) | lo
  return h >> 12, (h >> 6) & 0x3F, h & 0x3F


def sign_payload(kph: int, length: int = SIGN_PAYLOAD_LEN) -> bytes:
  p = bytearray(length)
  if kph > 0 and length >= 5:
    p[0] = 0x01             # speed limit sign, km/h
    p[1] = min(kph, 0xFF)
    p[4] = 0x10             # source/flags as sent by the camera
  return bytes(p)


def multi_frame(op: int, payload: bytes, channel: int = 0) -> list[bytes]:
  n = len(payload)
  frames = [bytes([0x80 | (channel << 4) | ((n >> 8) & 0x0F), n & 0xFF]) + bap_header(op, VZE_LSG, VZE_FCT_SIGN) + payload[:4]]
  pos, idx = 4, 0
  while pos < n:
    frames.append(bytes([0xC0 | (channel << 4) | (idx & 0x0F)]) + payload[pos:pos + 7])
    pos += 7
    idx += 1
  return frames


class VzeBapRelay:
  def __init__(self):
    self.kph = 0
    self.announced = 0        # camera starts with "no sign"
    self._mf = None           # camera multi-frame in progress: dict(sign, length, channel, pos)
    self._out: deque[tuple[int, bytes]] = deque()

  def set_limit(self, kph: float) -> None:
    self.kph = max(0, min(int(round(kph)), 0xFF))

  def feed(self, nanos: int, frame: bytes) -> None:
    """One camera BAP frame (0x17332110 from the camera side), in arrival order."""
    if len(frame) < 2:
      self._out.append((nanos, bytes(frame)))
      return
    d = bytearray(frame)
    if d[0] & 0x80 == 0:  # single frame
      _, lsg, fct = parse_header(d[0], d[1])
      if lsg == VZE_LSG and fct == VZE_FCT_SIGN and len(d) > 2:
        d[2:] = sign_payload(self.kph, len(d) - 2)
      elif lsg == VZE_LSG and fct == VZE_FCT_STATUS and len(d) > 2:
        d[2] &= ~VZE_STATUS_RESTRICTED & 0xFF
    elif d[0] & 0xC0 == 0x80:  # start of a multi frame
      if len(d) >= 4:
        _, lsg, fct = parse_header(d[2], d[3])
        mf = {'sign': lsg == VZE_LSG and fct == VZE_FCT_SIGN, 'length': ((d[0] & 0x0F) << 8) | d[1],
              'channel': (d[0] >> 4) & 0x3, 'pos': len(d) - 4}
        if mf['sign']:
          d[4:] = sign_payload(self.kph, mf['length'])[:len(d) - 4]
        self._mf = mf if mf['pos'] < mf['length'] else None
    else:  # continuation
      mf = self._mf
      if mf is not None and (d[0] >> 4) & 0x3 == mf['channel']:
        n = len(d) - 1
        if mf['sign']:
          seg = sign_payload(self.kph, mf['length'])[mf['pos']:mf['pos'] + n]
          d[1:1 + len(seg)] = seg
        mf['pos'] += n
        if mf['pos'] >= mf['length']:
          self._mf = None
    self._out.append((nanos, bytes(d)))

  def step(self, now_nanos: int) -> list[bytes]:
    """Called once per control cycle; returns at most one frame to send on the car side."""
    # announce a changed limit, but never inside one of the camera's multi frames
    if self.kph != self.announced and self._mf is None:
      for f in multi_frame(OP_STATUS, sign_payload(self.kph)):
        self._out.append((now_nanos, f))
      self.announced = self.kph

    while self._out and now_nanos - self._out[0][0] > MAX_AGE_NS:
      self._out.popleft()
    return [self._out.popleft()[1]] if self._out else []
