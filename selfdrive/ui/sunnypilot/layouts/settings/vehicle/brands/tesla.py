"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp

LKAS_MIN_SPEED_KMH = 23 # power-steering min speed above which the user can provide steer corrections
OEM_STEERING_MIN_KMH = 48 # Tesla Lane Assist min speed per the manual
KM_TO_MILE = 0.621371


class TeslaSettings(BrandSettings):
  def __init__(self):
    super().__init__()
    self.lkas_title = tr("OEM/LKAS Cooperative Steering (Beta)")
    self.lkas_steering_toggle = toggle_item_sp(self.lkas_title, "", param="TeslaLkasSteering")
    self.coop_steering_toggle = toggle_item_sp(tr("Emulated Cooperative Steering"), "", param="TeslaCoopSteering")
    self.items = [self.lkas_steering_toggle, self.coop_steering_toggle]

  def update_settings(self):
    is_metric = ui_state.is_metric
    unit = "km/h" if is_metric else "mph"

    display_value_lkas = LKAS_MIN_SPEED_KMH if is_metric else round(LKAS_MIN_SPEED_KMH * KM_TO_MILE)
    display_value_oem = OEM_STEERING_MIN_KMH if is_metric else round(OEM_STEERING_MIN_KMH * KM_TO_MILE)

    lkas_warning = tr(
        "Warning: May experience hard steering oscillations below {speed} {unit} during turns, " +
        "recommend disabling this feature if you experience these."
    ).format(speed=display_value_oem, unit=unit)

    lkas_desc = (
      f"{tr('Allows the driver to provide limited steering input while openpilot is engaged.')}<br>" +
      f"{tr('Only works above {speed} {unit}.').format(speed=display_value_lkas, unit=unit)}</b><br><br>" +
      f"{tr('When driving straight, it requires significant amount of force to correct the steering.')} " +
      f"{tr('However, steering is much lighter when correcting toward the center during turns and may also provide light vibration.')}<br>" +
      f"<b>{lkas_warning}"
    )

    coop_steering_desc = (
      f"{tr('Converts light steering input into a steering rotation.')} - " +
      f"{tr('It works at any speed, the faster you go the stiffer the steering gets.')}<br>" +
      f"{tr('It can co-exist with the {lkas_feature}, which helps to reduce disengagements on faster corrections.'
        ).format(lkas_feature=self.lkas_title)}"
    )

    enable_offroad_msg = tr("Enable \"Always Offroad\" in Device panel, or turn vehicle off to toggle.")
    if not ui_state.is_offroad():
      lkas_desc = f"<b>{enable_offroad_msg}</b><br><br>{lkas_desc}"
      coop_steering_desc = f"<b>{enable_offroad_msg}</b><br><br>{coop_steering_desc}"

    self.lkas_steering_toggle.set_description(lkas_desc)
    self.coop_steering_toggle.set_description(coop_steering_desc)

    self.lkas_steering_toggle.action_item.set_enabled(ui_state.is_offroad())
    self.coop_steering_toggle.action_item.set_enabled(ui_state.is_offroad())
