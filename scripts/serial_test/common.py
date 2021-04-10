import time
import ctypes
from ctypes import Structure, c_float, c_ulonglong

class PoseUpdate(Structure):
    _pack_ = 1
    _fields_ = [("time_us", c_ulonglong),
                ("x", c_float),
                ("y", c_float),
                ("z", c_float),
                ("roll", c_float),
                ("pitch", c_float),
                ("yaw", c_float)]

    def __repr__(self) -> str:
        values = ", ".join(f"{name}={value}"
                          for name, value in self._asdict().items())
        return f"<{self.__class__.__name__}: {values}>"

    def _asdict(self) -> dict:
        return {field[0]: getattr(self, field[0])
                for field in self._fields_}

class LandingCommand(Structure):
    _pack_ = 1
    _fields_ = [("time_us", c_ulonglong),
                ("x", c_float),
                ("y", c_float),
                ("z", c_float)]

    def __repr__(self) -> str:
        values = ", ".join(f"{name}={value}"
                          for name, value in self._asdict().items())
        return f"<{self.__class__.__name__}: {values}>"

    def _asdict(self) -> dict:
        return {field[0]: getattr(self, field[0])
                for field in self._fields_}

def get_us_from_epoch():
    time_us = int(time.time() * 1000 * 1000)
    return time_us

def serialize(obj_ctype):
    # cast the struct to a pointer to a char array
    pdata = ctypes.cast(ctypes.byref(obj_ctype), ctypes.POINTER(ctypes.c_char * ctypes.sizeof(obj_ctype)))
    # pdata.contents.raw
    return pdata

def deserialize(empty_obj_ctype, raw):
    ctypes.memmove(ctypes.pointer(empty_obj_ctype),raw, ctypes.sizeof(empty_obj_ctype))

def test():
    time_us = get_us_from_epoch()
    pose_update = PoseUpdate(time_us, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0)
    print("Pose Update:")
    print(pose_update)
    serialized_pose = serialize(pose_update)
    print("Raw Serialized Data:")
    print(serialized_pose.contents.raw)

    print("Attempting to deserialize from raw bytes")
    pose_update_new = PoseUpdate()
    deserialize(pose_update_new, serialized_pose.contents.raw)
    print("New Pose Update:")
    print(pose_update_new)

if __name__ == "__main__":
    test()