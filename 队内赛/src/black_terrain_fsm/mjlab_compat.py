"""Compatibility imports for the locally installed mjlab checkout.

The local Black task package currently has an unrelated syntax error in its
aggregating ``__init__.py``.  These modules are valid, so import the needed
modules by path without executing the broken package initializer.  Nothing in
the mjlab checkout or its checkpoints is modified.
"""

from __future__ import annotations

import importlib
import sys
import types
from pathlib import Path

import mjlab


def install_import_workaround() -> None:
    root = Path(mjlab.__file__).resolve().parent
    packages = {
        "mjlab.tasks": root / "tasks",
        "mjlab.tasks.velocity": root / "tasks" / "velocity",
        "mjlab.tasks.velocity.config": root / "tasks" / "velocity" / "config",
        "mjlab.tasks.velocity.config.black": (
            root / "tasks" / "velocity" / "config" / "black"
        ),
    }
    for name, path in packages.items():
        if not path.is_dir():
            raise RuntimeError(f"mjlab package path does not exist: {path}")
        module = types.ModuleType(name)
        module.__path__ = [str(path)]
        module.__package__ = name
        sys.modules[name] = module


def black_env_cfgs():
    install_import_workaround()
    return importlib.import_module("mjlab.tasks.velocity.config.black.env_cfgs")


def black_rl_cfg():
    install_import_workaround()
    return importlib.import_module("mjlab.tasks.velocity.config.black.rl_cfg")
