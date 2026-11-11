import unittest

from opendbc.sunnypilot.car.volkswagen.vze_bap import VzeBapRelay, bap_header, parse_header, sign_payload, OP_STATUS

MS = 1_000_000
h = bytes.fromhex

# BAP frames recorded from a North American camera with VZE coded (no sign readable) ...
NA_SIGN_HEARTBEAT = [h('8012385000000000'), h('c000000000000000'), h('c100000000000000')]
# ... and the same function from an EU A3 camera showing an 80 sign
EU_SIGN_80_HEARTBEAT = [h('8012385001500000'), h('c010000000000000'), h('c100000000000000')]
OTHER_FRAMES = [h('384f00'), h('8008384338 07ba80'.replace(' ', '')), h('c000000000'), h('38420300210 00405'.replace(' ', ''))]


def run(relay, frames, t0=0, limit=None):
  """feed frames 10 ms apart, stepping the relay once per 10 ms; returns everything it emitted"""
  out, t = [], t0
  for f in frames:
    if limit is not None:
      relay.set_limit(limit)
    relay.feed(t, f)
    out += relay.step(t)
    t += 10 * MS
  for _ in range(20):  # drain
    out += relay.step(t)
    t += 10 * MS
  return out, t


class TestVzeBap(unittest.TestCase):
  def test_header(self):
    self.assertEqual(bap_header(3, 0x21, 0x10), h('3850'))
    self.assertEqual(bap_header(OP_STATUS, 0x21, 0x10), h('4850'))
    self.assertEqual(parse_header(0x38, 0x50), (3, 0x21, 0x10))

  def test_payload(self):
    self.assertEqual(sign_payload(0), bytes(18))
    self.assertEqual(sign_payload(80)[:6], h('015000001000'))

  def test_no_limit_passes_camera_through_unchanged(self):
    relay = VzeBapRelay()
    out, _ = run(relay, NA_SIGN_HEARTBEAT + OTHER_FRAMES, limit=0)
    self.assertEqual(out, NA_SIGN_HEARTBEAT + OTHER_FRAMES)

  def test_rewrites_camera_sign_payload(self):
    relay = VzeBapRelay()
    relay.set_limit(80)
    relay.announced = 80  # isolate the rewrite from the change announcement
    out, _ = run(relay, NA_SIGN_HEARTBEAT)
    expected = [EU_SIGN_80_HEARTBEAT[0], h('c010000000000000'), EU_SIGN_80_HEARTBEAT[2]]
    self.assertEqual(out, expected)

  def test_other_functions_untouched(self):
    relay = VzeBapRelay()
    relay.set_limit(100)
    relay.announced = 100
    out, _ = run(relay, OTHER_FRAMES)
    self.assertEqual(out, OTHER_FRAMES)

  def test_announces_change_once(self):
    relay = VzeBapRelay()
    relay.set_limit(80)
    out = []
    for i in range(10):
      out += relay.step(i * 10 * MS)
    self.assertEqual(out, [h('8012485001500000'), h('c010000000000000'), h('c100000000000000')])
    self.assertEqual(sum((relay.step(t * MS) for t in range(200, 300, 10)), []), [])

  def test_announcement_waits_for_camera_multi_frame(self):
    relay = VzeBapRelay()
    relay.feed(0, OTHER_FRAMES[1])           # camera starts a multi frame (fct 0x03, 2 frames)
    relay.set_limit(50)
    out = relay.step(0)
    self.assertEqual(out, [OTHER_FRAMES[1]])
    self.assertEqual(relay.step(10 * MS), [])  # nothing injected mid-way
    relay.feed(20 * MS, OTHER_FRAMES[2])      # camera finishes it
    out = []
    for t in range(20, 100, 10):
      out += relay.step(t * MS)
    self.assertEqual(out[0], OTHER_FRAMES[2])
    self.assertEqual(out[1][:6], h('801248500132'))  # then the change, 0x32 = 50

  def test_clears_restricted_status(self):
    relay = VzeBapRelay()
    # camera status function 0x13: heartbeat (op3) and change (op4); only the 0x08 "restricted" bit is cleared
    frames = [h('385308 0000'.replace(' ', '')), h('485308 0000'.replace(' ', '')), h('385309 0000'.replace(' ', ''))]
    out, _ = run(relay, frames, limit=0)
    self.assertEqual(out, [h('3853000000'), h('4853000000'), h('3853010000')])

  def test_restricted_bit_only_touched_in_status_function(self):
    relay = VzeBapRelay()
    frames = [h('3854080000 01'.replace(' ', '')), h('384d10')]  # 0x14 and 0x0d carry 0x08/0x10 legitimately
    out, _ = run(relay, frames, limit=0)
    self.assertEqual(out, frames)

  def test_drops_stale_frames(self):
    relay = VzeBapRelay()
    for f in NA_SIGN_HEARTBEAT:
      relay.feed(0, f)
    self.assertEqual(relay.step(500 * MS), [])


if __name__ == '__main__':
  unittest.main()
