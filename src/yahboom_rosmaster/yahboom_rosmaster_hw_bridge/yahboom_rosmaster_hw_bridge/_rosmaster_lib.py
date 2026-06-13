"""Import-guarded access to Yahboom's stock Rosmaster_Lib Python driver.

On the real Jetson Orin NX Super the package is preinstalled (v3.3.9 at
/usr/local/lib/python3.10/dist-packages/Rosmaster_Lib-3.3.9-py3.10.egg);
on dev machines it is absent. Bridge nodes call `acquire_driver()`
lazily so importing them never fails when the library isn't present —
the node just logs a warning and refuses to drive hardware.

Only ONE process per host can hold /dev/myserial open at a time, so
each bridge node that calls `acquire_driver()` is responsible for
being the sole serial owner in its process — currently base_driver
in Phase 1, and (in Phase 2) the consolidated yahboom_bridge_node.
"""
from __future__ import annotations

import logging
import time
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


def acquire_driver(
    port: str = "/dev/myserial",
    car_type: int = 1,
    delay: float = 0.002,
    debug: bool = False,
    handshake_settle_s: float = 0.3,
) -> Optional[Any]:
    """Open a Rosmaster serial driver and run the receive thread.

    Returns the live instance on success, or None if Rosmaster_Lib is
    not importable (stub mode). Subsequent calls return the same
    singleton.

    On real hardware (the Orin), `get_version()` returns a positive
    float like 3.5 after a successful handshake; -1 indicates the
    receive thread didn't see a response packet within the settle
    window — wrong port, board off, or wrong car_type are the usual
    suspects.
    """
    global _DRIVER_SINGLETON
    if _DRIVER_SINGLETON is not None:
        return _DRIVER_SINGLETON

    lib = try_import_rosmaster_lib()
    if lib is None:
        logging.getLogger("yahboom_hw_bridge").warning(
            "Rosmaster_Lib not importable on this host (%s); running in stub mode.",
            _IMPORT_ERROR,
        )
        return None

    driver = lib.Rosmaster(car_type=car_type, com=port, delay=delay, debug=debug)
    driver.create_receive_threading()
    time.sleep(handshake_settle_s)
    # The constructor arg only stores the enum library-side; the STM32
    # keeps whatever profile it last had unless explicitly told. The
    # firmware uses the enum for BOTH command scaling and encoder
    # feedback, so skipping this made the bridge drive ~5x hot with
    # inverted odometry regardless of the launch's car_type.
    try:
        driver.set_car_type(int(car_type))
        time.sleep(0.1)
    except Exception as exc:  # noqa: BLE001
        logging.getLogger("yahboom_hw_bridge").warning(
            "set_car_type(%s) failed: %s", car_type, exc
        )
    _DRIVER_SINGLETON = driver
    return driver
