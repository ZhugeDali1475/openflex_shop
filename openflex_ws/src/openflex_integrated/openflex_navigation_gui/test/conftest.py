import os

import pytest
from PyQt5.QtCore import QSettings


os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("OPENFLEX_DISABLE_WEBENGINE", "1")


@pytest.fixture(autouse=True)
def isolate_gui_settings(tmp_path):
    QSettings.setDefaultFormat(QSettings.IniFormat)
    QSettings.setPath(QSettings.IniFormat, QSettings.UserScope, str(tmp_path / "settings"))
    yield
