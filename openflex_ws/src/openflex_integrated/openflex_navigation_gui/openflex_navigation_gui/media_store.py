"""In-memory compressed-image store selected through native Qt dialogs."""

from __future__ import annotations

import base64
import hashlib
import mimetypes
import uuid
from pathlib import Path

from PyQt5.QtCore import QBuffer, QByteArray, QIODevice
from PyQt5.QtGui import QImage
from PyQt5.QtWidgets import QFileDialog


class MediaStore:
    MAX_FILE_BYTES = 32 * 1024 * 1024
    MAX_FILES = 20

    def __init__(self, dialog_parent=None) -> None:
        self.dialog_parent = dialog_parent
        self._assets: dict[str, dict] = {}

    @staticmethod
    def _preview(data: bytes, mime_type: str) -> str:
        image = QImage.fromData(data)
        if image.isNull():
            return ""
        thumbnail = image.scaled(1280, 720, aspectRatioMode=1, transformMode=1)
        encoded = QByteArray()
        buffer = QBuffer(encoded)
        buffer.open(QIODevice.WriteOnly)
        thumbnail.save(buffer, "JPEG", 78)
        buffer.close()
        return "data:image/jpeg;base64," + base64.b64encode(bytes(encoded)).decode("ascii")

    def choose(self, multiple: bool = False) -> list[dict]:
        filters = "图片 (*.png *.jpg *.jpeg *.webp *.bmp);;所有文件 (*)"
        if multiple:
            selected, _ = QFileDialog.getOpenFileNames(
                self.dialog_parent, "选择图片", "", filters
            )
        else:
            selected, _ = QFileDialog.getOpenFileName(
                self.dialog_parent, "选择图片", "", filters
            )
            selected = [selected] if selected else []
        if len(selected) > self.MAX_FILES:
            raise ValueError(f"一次最多选择 {self.MAX_FILES} 张图片")

        result = []
        for filename in selected:
            path = Path(filename)
            data = path.read_bytes()
            if not data:
                raise ValueError(f"图片为空：{path.name}")
            if len(data) > self.MAX_FILE_BYTES:
                raise ValueError(f"图片超过 32 MiB：{path.name}")
            mime_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
            if QImage.fromData(data).isNull():
                raise ValueError(f"无法解码图片：{path.name}")
            asset_id = uuid.uuid4().hex
            stored = {
                "asset_id": asset_id,
                "original_name": path.name,
                "mime_type": mime_type,
                "compressed_size": len(data),
                "content_hash": "sha256:" + hashlib.sha256(data).hexdigest(),
                "preview_uri": self._preview(data, mime_type),
                "data": data,
            }
            self._assets[asset_id] = stored
            result.append({key: value for key, value in stored.items() if key != "data"})
        return result

    def bytes(self, asset_id: str) -> tuple[bytes, str]:
        asset = self._assets.get(str(asset_id))
        if asset is None:
            raise ValueError("图片资源不存在或已经释放")
        return asset["data"], asset["mime_type"]

    def clear(self) -> None:
        self._assets.clear()
