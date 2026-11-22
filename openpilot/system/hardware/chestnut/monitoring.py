import struct
import time

import usb1

import openpilot.cereal.messaging as messaging
from openpilot.cereal.services import SERVICE_LIST
from openpilot.common.hardware.usb import CHESTNUT_USB_PRODUCT, get_usb_state, is_chestnut_usb_id
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware.chestnut.status import CHESTNUT_POWER_LOW_DELAY, CHESTNUT_POWERED_VOLTAGE


class SupplyLog:
  """Reports a supply loss once, from whichever context reads the rail first."""

  def __init__(self):
    self.low = False
    self.low_since: float | None = None

  def update(self, voltage: int, current: int, fault: bool) -> None:
    if not (fault or voltage < CHESTNUT_POWERED_VOLTAGE):
      if self.low:
        cloudlog.info(f"chestnut supply ok again: {voltage} mV, fault={fault}")
      self.low = False
      self.low_since = None
      return

    if self.low_since is None:
      self.low_since = time.monotonic()
    if not self.low and time.monotonic() - self.low_since >= CHESTNUT_POWER_LOW_DELAY:
      self.low = True
      cloudlog.warning(f"chestnut supply unpowered for {CHESTNUT_POWER_LOW_DELAY:.0f}s: {voltage} mV, {current} mA, fault={fault}")


def read_chestnut_state(handle, gpu_state=None):
  msg = messaging.new_message('chestnutState')
  if gpu_state is not None:
    msg.chestnutState = gpu_state
  state = msg.chestnutState
  try:
    raw = handle.controlRead(0xC0, 0xC0, 0, 0, 5, timeout=100)
    state.supplyVoltage, state.supplyCurrent, state.supplyFault = struct.unpack('<Hh?', bytes(raw))
    raw = handle.controlRead(0xC0, 0xE4, 0xB450, 0, 1, timeout=100)
    state.pcieLtssm, = struct.unpack('B', bytes(raw))
    msg.valid = True
  except (usb1.USBError, struct.error):
    msg.valid = False
  return msg


def chestnut_state_thread(end_event):
  pm = messaging.PubMaster(['chestnutState'])
  sm = messaging.SubMaster(['chestnutGpuState'])
  supply_log = SupplyLog()
  with usb1.USBContext() as context:
    handle = None
    try:
      while not end_event.is_set():
        if handle is None:
          devices = [d for d in get_usb_state() if is_chestnut_usb_id(d['vendorId'], d['productId']) and
                     d['product'] == CHESTNUT_USB_PRODUCT]
          if len(devices) == 1:
            try:
              handle = context.openByVendorIDAndProductID(devices[0]['vendorId'], devices[0]['productId'], skip_on_error=True)
            except usb1.USBError:
              pass
        if handle is not None:
          sm.update(0)
          gpu_valid = sm.alive['chestnutGpuState'] and sm.valid['chestnutGpuState']
          msg = read_chestnut_state(handle, sm['chestnutGpuState'] if gpu_valid else None)
          if msg.valid:
            supply_log.update(msg.chestnutState.supplyVoltage, msg.chestnutState.supplyCurrent, msg.chestnutState.supplyFault)
          if not msg.valid:
            handle.close()
            handle = None
          msg.valid &= not sm.seen['chestnutGpuState'] or gpu_valid
          pm.send('chestnutState', msg)
        end_event.wait(1 / SERVICE_LIST['chestnutState'].frequency if handle is not None else 1.)
    finally:
      if handle is not None:
        handle.close()
