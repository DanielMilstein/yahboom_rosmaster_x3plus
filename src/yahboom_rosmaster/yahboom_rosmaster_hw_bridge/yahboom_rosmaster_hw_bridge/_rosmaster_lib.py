"""Import-guarded access to Yahboom's stock Rosmaster_Lib Python driver.

On the real Jetson Orin NX Super the package is preinstalled as part of
Yahboom's ROS 1 image; on Phase 0 dev machines it is absent. The stubs
in this package call `acquire_driver()` lazily so importing them never
fails when the library isn't present — the node just logs a warning
and refuses to drive hardware. Concrete protocol calls are wired in
Phase 1 once the lib is confirmed available on the Orin.
"""
from __future__ import annotations

import logging
from typing import Any, Optional

_DRIVER_SINGLETON: Optional[Any] = None
_IMPORT_ERROR: Optional[BaseException] = None


def try_import_rosmaster_lib() -> Optional[Any]:
    """Attempt to import Rosmaster_Lib. Returns the module on success, None on failure."""
    global _IMPORT_ERROR
    try:
        import Rosmaster_Lib  # type: ignore[import-not-found]
        return Rosmaster_Lib
    except Exception as exc:  # noqa: BLE001 — Yahboom's import may raise non-ImportError
        _IMPORT_ERROR = exc
        return None


def acquire_driver(port: str = "/dev/myserial", baud: int = 115200) -> Optional[Any]:
    """Return a singleton Rosmaster instance or None if the lib is missing.

    Phase 0: always returns None and logs once. Phase 1 wires the real
    serial handshake here.
    """
    global _DRIVER_SINGLETON
    if _DRIVER_SINGLETON is not None:
        return _DRIVER_SINGLETON
    lib = try_import_rosmaster_lib()
    if lib is None:
        logging.getLogger("yahboom_hw_bridge").warning(
            "Rosmaster_Lib not importable on this host (%s); "
            "running in stub mode. Phase 1 will wire the real driver on the Orin.",
            _IMPORT_ERROR,
        )
        return None
    # Phase 1: instantiate driver, open port, run handshake. Placeholder for now.
    logging.getLogger("yahboom_hw_bridge").warning(
        "Rosmaster_Lib imported but driver construction is not wired yet "
        "(port=%s, baud=%d). Running in stub mode.",
        port,
        baud,
    )
    return None
