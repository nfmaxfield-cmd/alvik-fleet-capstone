"""(e22, 13 Nov) Shared fixture for the fleet tests. Changelog: e22-2026-11-13 first version.

e22_old_ways: a test that describes how the run behaved before e22 (a person asked for a hand or a click; a standing
robot corrected at once from any sighting) pins those ways back on: MIRLT_SELF_RESCUE=ask and MIRLT_ID_GUARD=0, both
kept working behind their switches ("keep features, switch them off"). The e22 ways are tested in test_e22_no_standing.
"""
import sys

import pytest


@pytest.fixture
def e22_old_ways(monkeypatch):
    monkeypatch.setenv("MIRLT_SELF_RESCUE", "ask")          # for runs started as a child process
    monkeypatch.setenv("MIRLT_ID_GUARD", "0")
    fc = sys.modules.get("freeroam_core")
    if fc is not None:
        monkeypatch.setattr(fc, "SELF_RESCUE", False)
        monkeypatch.setattr(fc, "ID_GUARD", False)
    yield
