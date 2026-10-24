"""Bookmarks: named arm positions (firmware mm), kept in a YAML file."""
import os
import threading
import time
from pathlib import Path

import yaml


class Bookmarks:
    def __init__(self, path: Path):
        self.path = path
        self._lock = threading.Lock()
        self.items = []
        self._load()

    def _load(self):
        try:
            data = yaml.safe_load(self.path.read_text()) or {}
        except FileNotFoundError:
            data = {}
        items = []
        for b in data.get("bookmarks", []):
            try:
                items.append({"id": int(b["id"]), "name": str(b["name"]),
                              "xyz": [round(float(v), 1) for v in b["xyz"]][:3],
                              "created": float(b.get("created", 0))})
            except (KeyError, TypeError, ValueError):
                continue  # skip malformed entries rather than refusing to start
        self.items = items

    def _save(self):
        data = {"bookmarks": [
            {"id": b["id"], "name": b["name"], "xyz": b["xyz"],
             "created": round(b["created"], 3),
             "created_at": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(b["created"]))}
            for b in sorted(self.items, key=lambda b: b["created"])]}
        tmp = self.path.with_suffix(".yaml.tmp")
        tmp.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True))
        os.replace(tmp, self.path)  # atomic: never leaves a half-written file

    def snapshot(self):
        """Newest first."""
        with self._lock:
            return sorted((dict(b) for b in self.items), key=lambda b: -b["created"])

    def get(self, bid):
        with self._lock:
            return next((dict(b) for b in self.items if b["id"] == bid), None)

    def add(self, xyz, name=None):
        with self._lock:
            bid = max((b["id"] for b in self.items), default=0) + 1
            b = {"id": bid, "name": (name or f"Bookmark {bid}").strip()[:80],
                 "xyz": [round(float(v), 1) for v in xyz][:3], "created": time.time()}
            self.items.append(b)
            self._save()
            return dict(b)

    def delete(self, bid):
        with self._lock:
            self.items = [b for b in self.items if b["id"] != bid]
            self._save()

    def rename(self, bid, name):
        name = str(name).strip()[:80]
        if not name:
            return
        with self._lock:
            for b in self.items:
                if b["id"] == bid:
                    b["name"] = name
            self._save()

    def move(self, bid, xyz):
        with self._lock:
            for b in self.items:
                if b["id"] == bid:
                    b["xyz"] = [round(float(v), 1) for v in xyz][:3]
            self._save()
