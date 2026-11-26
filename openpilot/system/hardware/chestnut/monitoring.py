import os
import struct
import time
from collections.abc import Callable

import usb1

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.cereal.services import SERVICE_LIST
from openpilot.common.hardware.usb import CHESTNUT_USB_PRODUCT, get_usb_state, is_chestnut_usb_id
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware.chestnut.status import CHESTNUT_POWER_LOW_DELAY, CHESTNUT_POWERED_VOLTAGE

SUPPLY_LOG_PATH = "/data/chestnut_supply.log"


class SupplyLog:
  """Logs chestnut supply edges to swaglog and to a file that outlives the process."""

  def __init__(self, path: str = SUPPLY_LOG_PATH, model_reported: Callable[[], bool] | None = None):
    self.path = path
    self._model_reported = model_reported
    self._persist = True
    self.low = self._ended_low()
    self.low_since: float | None = None
    self.sustained = False
    self.level: str | None = None

  def update(self, voltage: int, current: int, fault: bool, car: str, ign: str, state: str) -> None:
    now = time.monotonic()
    if fault or voltage < CHESTNUT_POWERED_VOLTAGE:
      if self.low_since is None:
        self.low_since = now
      if not self.low:
        self.low = True
        self.sustained = False
        self.level = self._loss_level(state)
        before_model_start = state == "onroad" and self._model_reported is not None and not self._model_reported()
        self._emit(self.level, self._body("unpowered", voltage, current, fault, car, ign, state, before_model_start))
      elif not self.sustained and now - self.low_since >= CHESTNUT_POWER_LOW_DELAY:
        self.sustained = True
        self.level = self.level or self._loss_level(state)
        label = f"still unpowered after {CHESTNUT_POWER_LOW_DELAY:.0f}s"
        self._emit(self.level, self._body(label, voltage, current, fault, car, ign, state))
      return

    if self.low or self.low_since is not None:
      self.low = False
      self.low_since = None
      self.sustained = False
      self.level = None
      self._emit("INFO", self._body("ok again", voltage, None, fault, car, ign, state))

  @staticmethod
  def _loss_level(state: str) -> str:
    # the car cutting accessory power offroad is expected; losing it onroad is not
    return "INFO" if state == "offroad" else "WARNING"

  @staticmethod
  def _body(label: str, voltage: int, current: int | None, fault: bool, car: str, ign: str, state: str,
            before_model_start: bool = False) -> str:
    body = f"chestnut supply {label}: {voltage} mV"
    if current is not None:
      body += f", {current} mA"
    body += f", fault={fault}, car={car}, ign={ign}, state={state}"
    if before_model_start:
      body += ", before_model_start=1"
    return body

  def _emit(self, level: str, body: str) -> None:
    if level == "WARNING":
      cloudlog.warning(body)
    else:
      cloudlog.info(body)
    self._append(level, body)

  def _append(self, level: str, body: str) -> None:
    if not self._persist:
      return
    line = f"{time.strftime('%Y-%m-%d %H:%M:%S')} uptime={time.monotonic():.1f}s {level} {body}\n"
    try:
      with open(self.path, "a", encoding="utf-8") as f:
        f.write(line)
        f.flush()
        os.fsync(f.fileno())
    except OSError as e:
      self._persist = False
      cloudlog.error(f"chestnut supply log unwritable ({self.path}): {e}")

  def _ended_low(self) -> bool:
    try:
      with open(self.path, encoding="utf-8") as f:
        lines = [line for line in f if line.strip()]
    except OSError:
      return False
    return bool(lines) and ("supply unpowered" in lines[-1] or "still unpowered" in lines[-1])

  @staticmethod
  def car_state(sm) -> tuple[str, str, str]:
    car = "n/a"
    if sm.alive['peripheralState'] and sm['peripheralState'].pandaType != log.PandaState.PandaType.unknown:
      car = f"{sm['peripheralState'].voltage} mV"
    ign = "?/?"
    if sm.alive['pandaStates'] and len(sm['pandaStates']) > 0:
      ign = f"{int(sm['pandaStates'][0].ignitionLine)}/{int(sm['pandaStates'][0].ignitionCan)}"
    state = "unknown"
    if sm.alive['deviceState']:
      state = "onroad" if sm['deviceState'].started else "offroad"
    return car, ign, state


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
  sm = messaging.SubMaster(['chestnutGpuState', 'peripheralState', 'pandaStates', 'deviceState'])
  params = Params()
  supply_log = SupplyLog(model_reported=lambda: params.get("ChestnutActive") is not None)
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
            car, ign, state = supply_log.car_state(sm)
            supply_log.update(msg.chestnutState.supplyVoltage, msg.chestnutState.supplyCurrent, msg.chestnutState.supplyFault, car, ign, state)
          if not msg.valid:
            handle.close()
            handle = None
          msg.valid &= not sm.seen['chestnutGpuState'] or gpu_valid
          pm.send('chestnutState', msg)
        end_event.wait(1 / SERVICE_LIST['chestnutState'].frequency if handle is not None else 1.)
    finally:
      if handle is not None:
        handle.close()
