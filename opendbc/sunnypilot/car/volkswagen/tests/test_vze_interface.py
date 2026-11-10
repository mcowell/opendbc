import unittest

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.volkswagen.values import CAR
from opendbc.sunnypilot.car.interfaces import setup_interfaces
from opendbc.sunnypilot.car.volkswagen.vze_bap import VZE_BAP_ADDR

CAM_SIGN = [bytes.fromhex(x) for x in ('8012385000000000', 'c000000000000000', 'c100000000000000')]


def make_interface(enabled: bool):
  CI = interfaces[CAR.AUDI_A3_MK3]
  CP = CI.get_non_essential_params(CAR.AUDI_A3_MK3)
  CP_SP = CI.get_non_essential_params_sp(CP, CAR.AUDI_A3_MK3)
  setup_interfaces(CI, CP, CP_SP, [{"VolkswagenClusterSpeedLimit": enabled}])
  return CI(CP, CP_SP)


def run(ci, frames, as_candata: bool, kph: float = 80):
  CC = structs.CarControl().as_reader()
  CC_SP = structs.CarControlSP(speedLimitDisplay=kph / 3.6)
  out, t = [], 0
  for i in range(60):
    msgs = []
    if i % 5 == 0 and i // 5 < len(frames):
      f = frames[i // 5]
      # card passes plain (address, dat, src) tuples; tests elsewhere use CanData
      msgs = [CanData(VZE_BAP_ADDR, f, 2) if as_candata else (VZE_BAP_ADDR, f, 2)]
    ci.update([(t, msgs)])
    _, cans = ci.apply(CC, CC_SP, t)
    out += [bytes(d) for a, d, b in cans if a == VZE_BAP_ADDR and b == 0]
    t += 10_000_000
  return out


class TestVzeInterface(unittest.TestCase):
  def test_disabled_sends_nothing(self):
    self.assertEqual(run(make_interface(False), CAM_SIGN, as_candata=False), [])

  def test_relays_plain_tuples_and_candata(self):
    for as_candata in (False, True):
      out = run(make_interface(True), CAM_SIGN * 2, as_candata)
      # the limit is applied one cycle after the first camera frame, so a change announcement (opcode 4)
      # follows, and the camera's next heartbeat is rewritten to 80 km/h
      self.assertIn(bytes.fromhex('8012485001500000'), out, as_candata)
      self.assertEqual(out[-3:], [bytes.fromhex('8012385001500000'), bytes.fromhex('c010000000000000'),
                                  bytes.fromhex('c100000000000000')], as_candata)


if __name__ == '__main__':
  unittest.main()
