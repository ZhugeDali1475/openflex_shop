"""Optional QtWebEngine host for the embedded supermarket React application."""

import os
import mimetypes
from pathlib import Path
from urllib.parse import unquote

from PyQt5.QtCore import QFile, QIODevice, QUrl, Qt
from PyQt5.QtWidgets import QLabel, QVBoxLayout, QWidget

# QtWebEngine must be imported before QApplication/QCoreApplication exists.
# competition_window imports this module while the process is still starting.
WEBENGINE_DISABLED = os.environ.get("OPENFLEX_DISABLE_WEBENGINE", "") == "1"
try:
    if WEBENGINE_DISABLED:
        raise ImportError("QtWebEngine is disabled for this process")
    from PyQt5.QtWebChannel import QWebChannel
    from PyQt5.QtWebEngineCore import (
        QWebEngineUrlRequestJob,
        QWebEngineUrlScheme,
        QWebEngineUrlSchemeHandler,
    )
    from PyQt5.QtWebEngineWidgets import QWebEngineView
except ImportError as error:
    QWebChannel = None
    QWebEngineView = None
    QWebEngineUrlRequestJob = None
    QWebEngineUrlScheme = None
    QWebEngineUrlSchemeHandler = object
    WEBENGINE_IMPORT_ERROR = error
else:
    WEBENGINE_IMPORT_ERROR = None


WEB_SCHEME = b"supermarket"
WEB_ORIGIN = "supermarket://app/"


def _register_web_scheme() -> None:
    if QWebEngineUrlScheme is None:
        return
    scheme = QWebEngineUrlScheme(WEB_SCHEME)
    scheme.setSyntax(QWebEngineUrlScheme.Syntax.Host)
    scheme.setFlags(QWebEngineUrlScheme.SecureScheme)
    QWebEngineUrlScheme.registerScheme(scheme)


_register_web_scheme()


class WebAssetSchemeHandler(QWebEngineUrlSchemeHandler):
    """Serve one installed Vite build with explicit browser MIME types."""

    MIME_TYPES = {
        ".css": "text/css",
        ".html": "text/html",
        ".js": "application/javascript",
        ".json": "application/json",
        ".map": "application/json",
        ".svg": "image/svg+xml",
        ".wasm": "application/wasm",
    }

    def __init__(self, web_root: Path, data_root: Path | None = None, parent=None) -> None:
        super().__init__(parent)
        self.web_root = Path(web_root).resolve()
        self.data_root = Path(data_root).resolve() if data_root is not None else None

    def _image_target(self, relative: str) -> tuple[Path, str] | None:
        if self.data_root is None:
            return None
        parts = relative.strip("/").split("/")
        target: Path
        if len(parts) == 4 and parts[0] == "api" and parts[1] == "item-images" and parts[3] == "0.png":
            slot_id = parts[2]
            if not slot_id or any(char not in "0123456789-" for char in slot_id):
                return None
            target = self.data_root / "item_images" / slot_id / "0.png"
        elif len(parts) == 5 and parts[0] == "api" and parts[1] == "shelf-images" and parts[4] == "0.png":
            try:
                shelf_id = int(parts[2])
                face = int(parts[3])
            except ValueError:
                return None
            if shelf_id < 0 or face not in (0, 1):
                return None
            filename = "-x_0.png" if face == 0 else "+x_0.png"
            target = self.data_root / "shelf_images" / str(shelf_id) / filename
        elif len(parts) == 3 and parts[0] == "api" and parts[1] == "sku-images":
            sku = parts[2].strip()
            if not sku or sku in (".", ".."):
                return None
            target = self.data_root / "sku_images" / f"{sku}.png"
        else:
            return None
        try:
            target = target.resolve()
            target.relative_to(self.data_root)
        except (OSError, ValueError):
            return None
        return target, "image/png"

    def requestStarted(self, job) -> None:  # noqa: N802 - Qt callback name
        url = job.requestUrl()
        if url.host() != "app":
            job.fail(QWebEngineUrlRequestJob.UrlNotFound)
            return
        relative = unquote(url.path()).lstrip("/") or "index.html"
        image = self._image_target(relative)
        if image is not None:
            target, mime_type = image
            if not target.is_file():
                job.fail(QWebEngineUrlRequestJob.UrlNotFound)
                return
            device = QFile(str(target), job)
            if not device.open(QIODevice.ReadOnly):
                job.fail(QWebEngineUrlRequestJob.RequestFailed)
                return
            job.reply(mime_type.encode("ascii"), device)
            return
        try:
            target = (self.web_root / relative).resolve()
            target.relative_to(self.web_root)
        except (OSError, ValueError):
            job.fail(QWebEngineUrlRequestJob.RequestDenied)
            return
        if not target.is_file():
            job.fail(QWebEngineUrlRequestJob.UrlNotFound)
            return
        mime_type = self.MIME_TYPES.get(target.suffix.lower())
        if mime_type is None:
            mime_type = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        device = QFile(str(target), job)
        if not device.open(QIODevice.ReadOnly):
            job.fail(QWebEngineUrlRequestJob.RequestFailed)
            return
        job.reply(mime_type.encode("ascii"), device)


def _unavailable_page(message: str, parent=None) -> QWidget:
    page = QWidget(parent)
    layout = QVBoxLayout(page)
    label = QLabel(message)
    label.setAlignment(Qt.AlignCenter)
    label.setWordWrap(True)
    label.setStyleSheet("font-size: 15px; color: #5d6670; padding: 36px;")
    layout.addStretch(1)
    layout.addWidget(label)
    layout.addStretch(1)
    page.status_message = message
    return page


def create_scene_management_page(ros_node=None, parent=None) -> QWidget:
    """Create the WebEngine workspace, or a non-fatal dependency status page."""
    if WEBENGINE_IMPORT_ERROR is not None:
        return _unavailable_page(
            "场景管理页面需要 QtWebEngine。\n"
            "请安装：sudo apt install python3-pyqt5.qtwebengine\n\n"
            f"导入错误：{WEBENGINE_IMPORT_ERROR}\n\n"
            "2D 场景配准、导航和比赛控制仍可正常使用。",
            parent,
        )

    try:
        from ament_index_python.packages import get_package_share_directory

        index_path = (
            Path(get_package_share_directory("supermarket_scene_web"))
            / "web"
            / "index.html"
        )
    except Exception as error:
        return _unavailable_page(f"找不到 supermarket_scene_web：{error}", parent)
    if not index_path.is_file():
        return _unavailable_page(f"场景管理前端尚未构建：{index_path}", parent)

    from .web_bridge import WebSceneBridge

    view = QWebEngineView(parent)
    # With colcon --symlink-install, each installed asset can point back to
    # web/dist. Anchor traversal checks at the resolved build directory.
    configured_data_root = os.environ.get("SUPERMARKET_SCENE_DATA_ROOT", "").strip()
    if configured_data_root:
        data_root = Path(configured_data_root)
    else:
        database_path = os.environ.get("SUPERMARKET_SCENE_DB_PATH", "").strip()
        data_root = Path(database_path).resolve().parent / "data" if database_path else None
    scheme_handler = WebAssetSchemeHandler(index_path.resolve().parent, data_root, view)
    view.page().profile().installUrlSchemeHandler(WEB_SCHEME, scheme_handler)
    bridge = WebSceneBridge(ros_node, view)
    channel = QWebChannel(view.page())
    channel.registerObject("sceneBridge", bridge)
    view.page().setWebChannel(channel)
    view.setUrl(QUrl(WEB_ORIGIN + "index.html"))

    # Retain QObject ownership explicitly; otherwise Python wrappers can be
    # collected while JavaScript still holds the registered channel object.
    view.scene_bridge = bridge
    view.web_channel = channel
    view.web_scheme_handler = scheme_handler
    view.web_index_path = index_path
    return view
