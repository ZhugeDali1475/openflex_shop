from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import re
import threading
import time


class OperationStore:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.records = {}
        self.active_id = None
        self.cancel_event = threading.Event()
        if self.path.exists():
            for line in self.path.read_text().splitlines():
                record = json.loads(line)
                self.records[record["request_id"]] = record
            for record in self.records.values():
                if record["status"] in ("running", "unknown"):
                    self.active_id = record["request_id"]
                    record["status"] = "unknown"

    def persist(self, record):
        with self.path.open("a") as stream:
            stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
            stream.flush()
            os.fsync(stream.fileno())

    def begin(self, request_id, tool, arguments):
        if not isinstance(request_id, str) or re.fullmatch(r"[A-Za-z0-9_-]{1,80}", request_id) is None:
            raise ValueError("request_id must be 1-80 ASCII letters/digits/_/-")
        fingerprint = hashlib.sha256(json.dumps([tool, arguments], sort_keys=True).encode()).hexdigest()
        with self.lock:
            if request_id in self.records:
                record = self.records[request_id]
                if record["fingerprint"] != fingerprint:
                    raise ValueError("request_id already bound to different arguments")
                return deepcopy(record), False
            if self.active_id is not None:
                return {"status": "rejected_busy", "active_request_id": self.active_id, "success": False}, False
            record = {"request_id": request_id, "tool": tool, "arguments": arguments,
                      "fingerprint": fingerprint, "status": "running", "success": False,
                      "started_at": time.time(), "physical_verification": False, "events": []}
            self.persist(record)
            self.records[request_id] = record
            self.active_id = request_id
            self.cancel_event.clear()
            return deepcopy(record), True

    def event(self, request_id, kind, data):
        with self.lock:
            record = self.records[request_id]
            record["events"].append({"time": time.time(), "kind": kind, "data": data})
            self.persist(record)

    def finish(self, request_id, status, **result):
        with self.lock:
            record = self.records[request_id]
            record.update(result, status=status, finished_at=time.time())
            self.persist(record)
            if status != "unknown":
                self.active_id = None
            return deepcopy(record)

    def read(self, request_id):
        with self.lock:
            return deepcopy(self.records.get(request_id, {"status": "not_found", "success": False}))

    def cancel(self, request_id):
        with self.lock:
            if request_id != self.active_id:
                return {"status": "not_active", "success": False}
            self.cancel_event.set()
            return {"status": "cancel_requested", "request_id": request_id, "success": False,
                    "warning": "Request is not proof of stopped robot"}
