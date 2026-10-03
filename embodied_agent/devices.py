"""Read-only Windows/PCM diagnostics. Never changes permissions or endpoints."""
from __future__ import annotations

import math
import sys

import numpy as np

from .audio import select_input_device


def microphone_diagnostics(override=None, *, samplerate=16000, seconds=3) -> dict:
    import sounddevice as sd
    result = {"devices": list(sd.query_devices()), "hostapis": list(sd.query_hostapis()),
              "default_device": list(sd.default.device), "permissions": {}, "endpoints": []}
    if sys.platform == "win32":
        import winreg
        key = r"Software\Microsoft\Windows\CurrentVersion\CapabilityAccessManager\ConsentStore\microphone"
        for label, hive, suffix in (("user", winreg.HKEY_CURRENT_USER, ""),
                                    ("desktop_apps", winreg.HKEY_CURRENT_USER, r"\NonPackaged"),
                                    ("machine", winreg.HKEY_LOCAL_MACHINE, "")):
            try:
                with winreg.OpenKey(hive, key + suffix) as handle:
                    result["permissions"][label] = winreg.QueryValueEx(handle, "Value")[0]
            except OSError:
                result["permissions"][label] = "unavailable"
        try:
            import pythoncom
            from pycaw.pycaw import AudioUtilities
            pythoncom.CoInitialize()
            try:
                for device in AudioUtilities.GetAllDevices():
                    name = str(device.FriendlyName)
                    if "Realtek" not in name or "Active" not in str(device.state):
                        continue
                    entry = {"name": name, "state": str(device.state)}
                    try:
                        volume = device.EndpointVolume
                        entry.update(muted=bool(volume.GetMute()), level=float(volume.GetMasterVolumeLevelScalar()))
                    except Exception as error:
                        entry["error_type"] = type(error).__name__
                    result["endpoints"].append(entry)
            finally:
                pythoncom.CoUninitialize()
        except Exception as error:
            result["endpoint_error_type"] = type(error).__name__
    try:
        selected = select_input_device(sd, override, samplerate=samplerate)
        result["selected"] = selected
        samples = sd.rec(int(seconds * samplerate), samplerate=samplerate,
                         channels=1, dtype="int16", device=selected["index"], blocking=True)
        values = samples.astype(np.float64)
        peak = int(np.max(np.abs(values), initial=0))
        rms = float(np.sqrt(np.mean(values ** 2)))
        result["pcm"] = {"samples": len(samples), "seconds": seconds, "peak": peak, "rms": rms,
                         "rms_dbfs": 20 * math.log10(rms / 32768) if rms else None,
                         "near_silent": peak <= 8 and rms < 1, "raw_audio_saved": False}
    except Exception as error:
        result["error_type"] = type(error).__name__
    return result
