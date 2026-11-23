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
Sign payload (18 bytes): three 6-byte slots [type, value, cond, 0, source, 0]; all zeros = no sign. The cluster's
Traffic signs page draws up to three signs side by side; other views show one. Type: high nibble = style (0 EU, 3 US,
4 Canada), low nibble = sign (1 speed limit, 5 no passing, 6 end of no passing, 7 no passing for trucks, d end of all
restrictions; 6, 7 and d exist in the EU style only). The cluster hides a sign whose style differs from the others.
Condition 0x0c = school plate. Source 0x10 as sent by an EU A3 camera (any source value draws the same).
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
SLOT_LEN = 6
STYLE_EU = 0x00
STYLE_CANADA = 0x40
COND_SCHOOL = 0x0c  # condition plate under/over the sign: school zone
NO_PASSING_SIGNS = (0x5, 0x9)            # the cluster draws both the same: no passing
EU_ONLY_SIGNS = (0x6, 0xa, 0x7, 0xb, 0xd, 0xe)  # end of no passing, no passing for trucks, end of all restrictions
MAX_AGE_NS = 200_000_000  # drop camera frames that waited longer than this (e.g. before panda engaged the relay)


def bap_header(op: int, lsg: int, fct: int) -> bytes:
  h = (op << 12) | (lsg << 6) | fct
  return bytes([h >> 8, h & 0xFF])


def parse_header(hi: int, lo: int) -> tuple[int, int, int]:
  h = (hi << 8) | lo
  return h >> 12, (h >> 6) & 0x3F, h & 0x3F


def sign_payload(kph: int, length: int = SIGN_PAYLOAD_LEN, style: int = STYLE_EU, extra: bytes = b'', cond: int = 0) -> bytes:
  """openpilot's speed limit in slot 1 (none when kph is 0) with condition plate cond (0 none, COND_SCHOOL), followed by
  extra slots"""
  p = bytearray(max(length, SIGN_PAYLOAD_LEN))
  if kph > 0:
    p[0] = style | 0x01     # speed limit sign
    p[1] = min(kph, 0xFF)
    p[2] = cond
    p[4] = 0x10             # source/flags as sent by the camera
  p[SLOT_LEN:SLOT_LEN + len(extra)] = extra[:SIGN_PAYLOAD_LEN - SLOT_LEN]
  return bytes(p[:length])


def passing_slots(camera_payload: bytes, style: int = STYLE_EU) -> bytes:
  """The camera's own no-passing signs (any slot), restyled to match slot 1, for slots 2 and 3. Speed limits are
  dropped (slot 1 is openpilot's), as are signs the target style does not have."""
  out = bytearray()
  for i in range(0, len(camera_payload) - SLOT_LEN + 1, SLOT_LEN):
    slot = bytearray(camera_payload[i:i + SLOT_LEN])
    sign = slot[0] & 0x0F
    if slot[0] == 0 or not (sign in NO_PASSING_SIGNS or (sign in EU_ONLY_SIGNS and style == STYLE_EU)):
      continue
    slot[0] = style | sign
    out += slot
  return bytes(out[:SIGN_PAYLOAD_LEN - SLOT_LEN])


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
  def __init__(self, style: int = STYLE_EU, pass_no_passing: bool = False):
    self.style = style
    self.pass_no_passing = pass_no_passing
    self.kph = 0
    self.cond = 0             # condition plate on openpilot's sign
    self.extra = b''          # slots 2-3: the camera's no-passing sign, when passed through
    self.announced = bytes(SIGN_PAYLOAD_LEN)  # camera starts with "no sign"
    self._mf = None           # camera multi-frame in progress: dict(sign, length, channel, pos, payload, camera)
    self._out: deque[tuple[int, bytes]] = deque()

  def set_limit(self, kph: float, school_zone: bool = False) -> None:
    self.kph = max(0, min(int(round(kph)), 0xFF))
    self.cond = COND_SCHOOL if school_zone and self.kph else 0

  def payload(self, length: int = SIGN_PAYLOAD_LEN) -> bytes:
    return sign_payload(self.kph, length, self.style, self.extra, self.cond)

  def _camera_sign(self, camera_payload: bytes) -> None:
    """A complete sign payload from the camera (before rewriting)"""
    if self.pass_no_passing:
      self.extra = passing_slots(camera_payload, self.style)

  def feed(self, nanos: int, frame: bytes) -> None:
    """One camera BAP frame (0x17332110 from the camera side), in arrival order."""
    if len(frame) < 2:
      self._out.append((nanos, bytes(frame)))
      return
    d = bytearray(frame)
    if d[0] & 0x80 == 0:  # single frame
      _, lsg, fct = parse_header(d[0], d[1])
      if lsg == VZE_LSG and fct == VZE_FCT_SIGN and len(d) > 2:
        camera = bytes(d[2:])
        d[2:] = self.payload(len(d) - 2)
        self._camera_sign(camera)
      elif lsg == VZE_LSG and fct == VZE_FCT_STATUS and len(d) > 2:
        d[2] &= ~VZE_STATUS_RESTRICTED & 0xFF
    elif d[0] & 0xC0 == 0x80:  # start of a multi frame
      if len(d) >= 4:
        _, lsg, fct = parse_header(d[2], d[3])
        mf = {'sign': lsg == VZE_LSG and fct == VZE_FCT_SIGN, 'length': ((d[0] & 0x0F) << 8) | d[1],
              'channel': (d[0] >> 4) & 0x3, 'pos': len(d) - 4}
        if mf['sign']:
          # one payload for the whole message, so a change mid-way cannot mix two payloads
          mf['payload'], mf['camera'] = self.payload(mf['length']), bytearray(d[4:])
          d[4:] = mf['payload'][:len(d) - 4]
        self._mf = mf if mf['pos'] < mf['length'] else None
        if mf['sign'] and self._mf is None:
          self._camera_sign(bytes(mf['camera']))
    else:  # continuation
      mf = self._mf
      if mf is not None and (d[0] >> 4) & 0x3 == mf['channel']:
        n = len(d) - 1
        if mf['sign']:
          mf['camera'] += d[1:]
          seg = mf['payload'][mf['pos']:mf['pos'] + n]
          d[1:1 + len(seg)] = seg
        mf['pos'] += n
        if mf['pos'] >= mf['length']:
          self._mf = None
          if mf['sign']:
            self._camera_sign(bytes(mf['camera'][:mf['length']]))
    self._out.append((nanos, bytes(d)))

  def step(self, now_nanos: int) -> list[bytes]:
    """Called once per control cycle; returns at most one frame to send on the car side."""
    # announce a changed sign payload, but never inside one of the camera's multi frames
    payload = self.payload()
    if payload != self.announced and self._mf is None:
      for f in multi_frame(OP_STATUS, payload):
        self._out.append((now_nanos, f))
      self.announced = payload

    while self._out and now_nanos - self._out[0][0] > MAX_AGE_NS:
      self._out.popleft()
    return [self._out.popleft()[1]] if self._out else []
