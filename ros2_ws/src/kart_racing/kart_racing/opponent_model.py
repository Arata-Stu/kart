"""ROS-free ctypes adapter to kart's existing C++ GP; no compilation at runtime.

GP d(s)/v(s) describes measured historical behaviour, NOT current visibility or
elapsed-time uncertainty. Only real measurements may be passed to observe().
Library absence inhibits learned use without disabling simple/static planning.
"""
import ctypes
import math
import os
from pathlib import Path
import threading


LIBRARY_ENV = 'KART_RACING_OPPONENT_MODEL_LIBRARY'
_NAMES = ('libkart_racing_opponent_model.so', 'libkart_racing_opponent_model.dylib',
          'kart_racing_opponent_model.dll')


class _Prediction(ctypes.Structure):
    _fields_ = [('d', ctypes.c_double), ('v', ctypes.c_double),
                ('d_sigma', ctypes.c_double), ('v_sigma', ctypes.c_double), ('valid', ctypes.c_int32)]


def _invalid():
    return dict(d=0., v=0., d_sigma=1., v_sigma=1., valid=False)


def _finite(values):
    return all(type(value) in (int, float) and math.isfinite(value) for value in values)


def _candidates(library_path):
    explicit = library_path if library_path is not None else os.environ.get(LIBRARY_ENV)
    if explicit is not None:
        path = Path(explicit)
        if not path.is_absolute():
            raise ValueError('Explicit opponent model library path must be absolute')
        return [str(path)]
    prefixes = [Path(path) for path in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep) if path]
    prefixes.extend(Path(__file__).resolve().parents)
    candidates = [str(prefix / 'lib' / name) for prefix in prefixes for name in _NAMES
                  if (prefix / 'lib' / name).is_file()]
    # Let the platform loader inspect its configured paths; never invoke a
    # compiler, subprocess, torch, ROS, or a network download from this adapter.
    return list(dict.fromkeys(candidates + list(_NAMES)))


def _configure(library):
    library.kart_opponent_model_abi_version.argtypes = []
    library.kart_opponent_model_abi_version.restype = ctypes.c_uint32
    if library.kart_opponent_model_abi_version() != 1:
        raise ValueError('Opponent model native ABI mismatch')
    definitions = {
        'create': ([ctypes.c_double], ctypes.c_void_p),
        'destroy': ([ctypes.c_void_p], None),
        'clear': ([ctypes.c_void_p], ctypes.c_int32),
        'observe': ([ctypes.c_void_p, ctypes.c_uint64] + [ctypes.c_double] * 6, ctypes.c_int32),
        'ready': ([ctypes.c_void_p], ctypes.c_int32),
        'coverage': ([ctypes.c_void_p], ctypes.c_double),
        'predict': ([ctypes.c_void_p, ctypes.c_double, ctypes.POINTER(_Prediction)], ctypes.c_int32),
    }
    for name, (arguments, result) in definitions.items():
        function = getattr(library, 'kart_opponent_model_' + name)
        function.argtypes, function.restype = arguments, result
    return library


class OpponentModel:
    def __init__(self, length, library_path=None):
        self._lock, self._library, self._handle = threading.RLock(), None, None
        self.available, self.reason, self.library_path = False, 'native_library_unavailable', ''
        self.detail = ''
        if not _finite([length]) or not 3 <= length <= 20000:
            raise ValueError('course length must be finite in [3,20000] metres')
        self.length = float(length)
        try:
            candidates = _candidates(library_path)
        except (TypeError, ValueError) as error:
            self.detail = str(error)[:512]
            return
        for candidate in candidates:
            try:
                library = _configure(ctypes.CDLL(candidate))
                handle = library.kart_opponent_model_create(self.length)
                if not handle:
                    raise ValueError('native model construction failed')
                self._library, self._handle = library, handle
                self.available, self.reason, self.library_path = True, '', candidate
                self.detail = ''
                break
            except (OSError, AttributeError, TypeError, ValueError) as error:
                self.detail = str(error)[:512]

    def observe(self, identity, time, s, d, v, d_variance, v_variance):
        """Feed a real same-identity measurement, with unwrapped along-course s.

        A gap alone preserves a mature profile. Identity/time/progress/profile
        discontinuities retain the C++ model's reset policy. No synthetic update
        or visibility timeout is silently applied here.
        """
        if (type(identity) is not int or not 0 <= identity < 2**64
                or not _finite([time, s, d, v, d_variance, v_variance])
                or not 0 <= v <= 20 or not 0 <= d_variance <= .25 or not 0 <= v_variance <= 4):
            return False
        with self._lock:
            if not self.available:
                return False
            return bool(self._library.kart_opponent_model_observe(
                self._handle, identity, time, s, d, v, d_variance, v_variance))

    def ready(self):
        with self._lock:
            return bool(self.available and self._library.kart_opponent_model_ready(self._handle))

    def coverage(self):
        with self._lock:
            if not self.available:
                return 0.
            value = float(self._library.kart_opponent_model_coverage(self._handle))
            return value if math.isfinite(value) and 0 <= value <= 1 else 0.

    def predict(self, station):
        if not _finite([station]):
            return _invalid()
        with self._lock:
            if not self.available:
                return _invalid()
            value = _Prediction()
            status = self._library.kart_opponent_model_predict(self._handle, station, ctypes.byref(value))
            if status != 0 or not _finite([value.d, value.v, value.d_sigma, value.v_sigma]):
                return _invalid()
            return dict(d=value.d, v=value.v, d_sigma=value.d_sigma, v_sigma=value.v_sigma, valid=bool(value.valid))

    def clear(self):
        with self._lock:
            return bool(self.available and self._library.kart_opponent_model_clear(self._handle))

    def close(self):
        with self._lock:
            handle, self._handle = self._handle, None
            self.available, self.reason = False, 'native_model_closed'
            self.detail = ''
            if handle and self._library is not None:
                self._library.kart_opponent_model_destroy(handle)

    def __del__(self):
        try:
            self.close()
        except (AttributeError, TypeError):
            pass  # Partially constructed instances / interpreter shutdown.
