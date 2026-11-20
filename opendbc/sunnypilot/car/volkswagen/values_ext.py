"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import IntFlag


class VolkswagenSafetyFlagsSP:
  # panda: allow VZE_01/VZE_02 TX on bus 0 and block the camera's copies (MQB only)
  VZE = 1


class VolkswagenFlagsSP(IntFlag):
  # show openpilot's speed limit as the traffic sign in the instrument cluster (MQB only)
  VZE_DISPLAY = 1
  # draw it as the Canadian MAXIMUM sign (BAP type 0x41) instead of the EU round sign
  VZE_CANADA = 2
  # also show the camera's own (map) no-passing sign next to it
  VZE_NO_PASSING = 4
