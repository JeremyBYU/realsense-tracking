import time
import ctypes
from ctypes import Structure, c_float, c_ulonglong, Array, c_ubyte


START_BYTE1 = 0x1B
START_BYTE2 = 0xFE


################################################################################
# A simple helper class
class StructHelper(object):
    def __get_value_str(self, name, fmt='{}'):
        val = getattr(self, name)
        if isinstance(val, Array):
            val = list(val)
        return fmt.format(val)

    def __str__(self):
        result = '{}:\n'.format(self.__class__.__name__)
        maxname = max(len(name) for name, type_ in self._fields_)
        for name, type_ in self._fields_:
            value = getattr(self, name)
            result += ' {name:<{width}}: {value}\n'.format(
                    name = name,
                    width = maxname,
                    value = self.__get_value_str(name),
                    )
        return result

    def __repr__(self):
        return '{name}({fields})'.format(
                name = self.__class__.__name__,
                fields = ', '.join(
                    '{}={}'.format(name, self.__get_value_str(name, '{!r}')) for name, _ in self._fields_)
                )

    @classmethod
    def _typeof(cls, field):
        """Get the type of a field
        Example: A._typeof(A.fld)
        Inspired by stackoverflow.com/a/6061483
        """
        for name, type_ in cls._fields_:
            if getattr(cls, name) is field:
                return type_
        raise KeyError

    @classmethod
    def read_from(cls, f):
        result = cls()
        if f.readinto(result) != sizeof(cls):
            raise EOFError
        return result

    def get_bytes_array(self):
        """Get raw byte string of this structure
        ctypes.Structure implements the buffer interface, so it can be used
        directly anywhere the buffer interface is implemented.
        https://stackoverflow.com/q/1825715
        """

        # Works for either Python2 or Python3
        return bytearray(self)

    def get_bytes(self):
        """Get buffer representation
        """

        # Works only on Python3
        return bytes(self)


################################################################################

class PoseUpdate(Structure, StructHelper):
    _pack_ = 1
    _fields_ = [("time_us", c_ulonglong),
                ("x", c_float),
                ("y", c_float),
                ("z", c_float),
                ("roll", c_float),
                ("pitch", c_float),
                ("yaw", c_float)]

    def __init__(self, time_us=0, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        super(PoseUpdate, self).__init__(time_us, x, y, z, roll, pitch, yaw)

class LandingCommand(Structure, StructHelper):
    _pack_ = 1
    _fields_ = [("time_us", c_ulonglong),
                ("x", c_float),
                ("y", c_float),
                ("z", c_float)]

    def __init__(self, time_us=0, x=0.0, y=0.0, z=0.0):
        super(LandingCommand, self).__init__(time_us, x, y, z)

PU_SIZE = ctypes.sizeof(PoseUpdate)
LC_SIZE = ctypes.sizeof(LandingCommand)

class MessagePoseUpdate(Structure, StructHelper):
    _pack = 1
    _fields_ = [("start_byte1", c_ubyte),
                ("start_byte2", c_ubyte),
                ("data_size", c_ubyte),
                ("msg_type", c_ubyte),
                ("pose_update", PoseUpdate),
                ]

    def __init__(self, start_byte1=START_BYTE1, start_byte2=START_BYTE2, data_size=PU_SIZE, msg_type=1, pose_update=PoseUpdate()):
        super(MessagePoseUpdate, self).__init__(start_byte1, start_byte2, data_size, msg_type, pose_update)

class MessageLandingCommand(Structure, StructHelper):
    _pack = 1
    _fields_ = [("start_byte1", c_ubyte),
                ("start_byte2", c_ubyte),
                ("data_size", c_ubyte),
                ("msg_type", c_ubyte),
                ("landing_command", LandingCommand),
                ]
    
    def __init__(self, start_byte1=START_BYTE1, start_byte2=START_BYTE2, data_size=LC_SIZE, msg_type=2, landing_command=LandingCommand()):
        super(MessageLandingCommand, self).__init__(start_byte1, start_byte2, data_size, msg_type, landing_command)

def get_us_from_epoch():
    time_us = int(time.time() * 1000 * 1000)
    return time_us

def serialize(obj_ctype):
    # cast the struct to a pointer to a char array
    pdata = ctypes.cast(ctypes.byref(obj_ctype), ctypes.POINTER(ctypes.c_char * ctypes.sizeof(obj_ctype)))
    # pdata.contents.raw
    return pdata

# def deserialize(empty_obj_ctype, raw):
#     ctypes.memmove(ctypes.pointer(empty_obj_ctype),raw, ctypes.sizeof(empty_obj_ctype))

def test():
    time_us = get_us_from_epoch()
    pose_update = PoseUpdate(time_us, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0)
    print("First Pose:")
    print(pose_update)
    print("Raw Serialized Data:")
    print(pose_update.get_bytes())

    print("Attempting to deserialize from raw bytes")
    # pose_update_new = PoseUpdate()
    buf = pose_update.get_bytes()
    pose_update_new = PoseUpdate.from_buffer_copy(buf)
    print("New Pose:")
    print(pose_update_new)

    print("Testing Full Messages")
    message_pose_update = MessagePoseUpdate(pose_update=pose_update)
    print(message_pose_update)
    print("Raw Serialized Data:")
    buf = message_pose_update.get_bytes()
    print(buf)
    print("Deserialized:")
    message_pose_update_new = MessagePoseUpdate.from_buffer_copy(buf)
    print(message_pose_update_new)


if __name__ == "__main__":
    test()