"""Filesystem helpers for runs under sudo."""

import os
from pathlib import Path


def give_to_sudo_user(path: Path) -> None:
    """Files written under sudo belong to root; hand them back to the user who ran sudo."""
    uid, gid = os.environ.get("SUDO_UID"), os.environ.get("SUDO_GID")
    if uid is None or gid is None or os.geteuid() != 0:
        return
    os.chown(path, int(uid), int(gid))


def make_dirs_for_user(path: Path) -> None:
    """Like mkdir -p, but every folder it creates is handed back to the sudo user."""
    missing = []
    current = path
    while not current.exists():
        missing.append(current)
        current = current.parent
    for folder in reversed(missing):
        folder.mkdir(exist_ok=True)
        give_to_sudo_user(folder)
