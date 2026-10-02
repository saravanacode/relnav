"""Filesystem helpers for runs under sudo."""

import os
from pathlib import Path


def give_to_sudo_user(path: Path) -> None:
    """Files written under sudo belong to root; hand them back to the user who ran sudo."""
    uid, gid = os.environ.get("SUDO_UID"), os.environ.get("SUDO_GID")
    if uid is None or gid is None or os.geteuid() != 0:
        return
    os.chown(path, int(uid), int(gid))
