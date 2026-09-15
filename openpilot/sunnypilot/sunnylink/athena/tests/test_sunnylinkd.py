"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from unittest import mock

from openpilot.sunnypilot.sunnylink.athena import sunnylinkd
from openpilot.common.test import OpenpilotTestCase


class TestSunnylinkdMethods(OpenpilotTestCase):
  def setup_method(self):
    self.saved_params = []

    def mock_save_param(key, value, compression=False):
      self.saved_params.append((key, value, compression))

    p = mock.patch.object(sunnylinkd, 'save_param_from_base64_encoded_string', mock_save_param)
    p.start()
    self.addCleanup(p.stop)

    # Mock params with IsEngaged=False by default
    self.mock_params = mock.MagicMock()
    self.mock_params.get_bool.return_value = False
    p = mock.patch.object(sunnylinkd, 'params', self.mock_params)
    p.start()
    self.addCleanup(p.stop)

  def test_saveParams_blocked(self):
    blocked_params = {
      "GithubUsername": "attacker",
      "GithubSshKeys": "ssh-rsa attacker_key",
      "SshEnabled": "1",
      "AdbEnabled": "1",
      "DoReboot": "1",
    }

    sunnylinkd.saveParams(blocked_params)

    assert len(self.saved_params) == 0

  def test_saveParams_allowed(self):
    allowed_params = {
      "DisengageOnAccelerator": "1",
      "IsMetric": "1",
    }

    sunnylinkd.saveParams(allowed_params)

    # verify content
    assert len(self.saved_params) == 2
    keys_saved = [p[0] for p in self.saved_params]
    assert "DisengageOnAccelerator" in keys_saved
    assert "IsMetric" in keys_saved

  def test_saveParams_mixed(self):
    mixed_params = {
      "GithubUsername": "attacker",
      "IsMetric": "1",
    }

    sunnylinkd.saveParams(mixed_params)

    # should save allowed one
    assert len(self.saved_params) == 1
    assert self.saved_params[0][0] == "IsMetric"
    assert self.saved_params[0][1] == "1"

  def test_saveParams_critical_blocked_when_engaged(self):
    """Schema-gated (not_engaged/offroad_only) params are blocked while engaged"""
    self.mock_params.get_bool.side_effect = lambda key: key == "IsEngaged"

    sunnylinkd.saveParams({
      "Mads": "1",
      "AlphaLongitudinalEnabled": "1",
      "OffroadMode": "1",
    })

    assert len(self.saved_params) == 0

  def test_saveParams_noncritical_allowed_when_engaged(self):
    """Non-critical params (no engaged gate) are allowed while engaged"""
    self.mock_params.get_bool.side_effect = lambda key: key == "IsEngaged"

    sunnylinkd.saveParams({
      "CameraOffset": "1",
      "DisengageOnAccelerator": "1",
      "DynamicExperimentalControl": "1",
    })

    assert len(self.saved_params) == 3

  def test_engaged_blocked_derived_from_schema(self):
    """ENGAGED_BLOCKED mirrors settings_ui.json gates; allowlist covers the rest"""
    assert sunnylinkd.ALLOWED_REMOTE is not None
    assert sunnylinkd.ENGAGED_BLOCKED is not None
    assert "OffroadMode" in sunnylinkd.ENGAGED_BLOCKED
    assert "Mads" in sunnylinkd.ENGAGED_BLOCKED
    assert "AlphaLongitudinalEnabled" in sunnylinkd.ENGAGED_BLOCKED
    assert "CameraOffset" not in sunnylinkd.ENGAGED_BLOCKED
    assert "DisengageOnAccelerator" not in sunnylinkd.ENGAGED_BLOCKED
    # never remotely writable: not in schema or blocked:true
    assert "DoReboot" not in sunnylinkd.ALLOWED_REMOTE
    assert "GithubUsername" not in sunnylinkd.ALLOWED_REMOTE
    assert "SshEnabled" not in sunnylinkd.ALLOWED_REMOTE
    assert "AdbEnabled" not in sunnylinkd.ALLOWED_REMOTE

  def test_saveParams_safety_critical_blocked_when_engaged(self):
    """Safety-critical params are blocked while engaged"""
    self.mock_params.get_bool.side_effect = lambda key: key == "IsEngaged"

    sunnylinkd.saveParams({
      "DoReboot": "1",
      "DoShutdown": "1",
      "DoUninstall": "1",
      "ForcePowerDown": "1",
      "OffroadMode": "1",
    })

    assert len(self.saved_params) == 0

  def test_saveParams_unknown_blocked_when_not_engaged(self):
    """Non-schema params are never remotely writable, even disengaged"""
    sunnylinkd.saveParams({
      "DoReboot": "1",
      "DoShutdown": "1",
      "DoUninstall": "1",
      "ForcePowerDown": "1",
      "GithubUsername": "attacker",
    })

    assert len(self.saved_params) == 0

  def test_saveParams_allowed_when_not_engaged(self):
    """In-schema params are allowed when not engaged"""
    sunnylinkd.saveParams({
      "OffroadMode": "1",
      "Mads": "1",
      "AlphaLongitudinalEnabled": "1",
      "CameraOffset": "0.1",
      "DisengageOnAccelerator": "1",
    })

    assert len(self.saved_params) == 5
