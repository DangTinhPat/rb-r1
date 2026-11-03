"""Run the real dashboard script against deterministic DOM/network/timer mocks."""
from pathlib import Path
import shutil
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which('node'), 'Node.js required for browser regression checks')
class DashboardBrowserTests(unittest.TestCase):
    def test_outage_reload_and_command_races(self):
        subprocess.run(['node', str(ROOT / 'tests/fixtures/dashboard_browser_tests.js'),
                        str(ROOT / 'web/index.html')], check=True, timeout=10)
