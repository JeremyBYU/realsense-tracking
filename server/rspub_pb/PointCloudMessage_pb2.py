# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: PointCloudMessage.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='PointCloudMessage.proto',
  package='rspub_pb',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=_b('\n\x17PointCloudMessage.proto\x12\x08rspub_pb\"|\n\x11PointCloudMessage\x12\x0f\n\x07pc_data\x18\x01 \x01(\x0c\x12\x10\n\x08n_points\x18\x02 \x01(\x05\x12\x13\n\x0bhardware_ts\x18\x03 \x01(\x01\x12\x0b\n\x03\x62pp\x18\x04 \x01(\x05\x12\x0e\n\x06\x66ormat\x18\x05 \x01(\x05\x12\x12\n\ncolor_data\x18\x06 \x01(\x0c\x62\x06proto3')
)




_POINTCLOUDMESSAGE = _descriptor.Descriptor(
  name='PointCloudMessage',
  full_name='rspub_pb.PointCloudMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='pc_data', full_name='rspub_pb.PointCloudMessage.pc_data', index=0,
      number=1, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=_b(""),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='n_points', full_name='rspub_pb.PointCloudMessage.n_points', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='hardware_ts', full_name='rspub_pb.PointCloudMessage.hardware_ts', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bpp', full_name='rspub_pb.PointCloudMessage.bpp', index=3,
      number=4, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='format', full_name='rspub_pb.PointCloudMessage.format', index=4,
      number=5, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='color_data', full_name='rspub_pb.PointCloudMessage.color_data', index=5,
      number=6, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=_b(""),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=37,
  serialized_end=161,
)

DESCRIPTOR.message_types_by_name['PointCloudMessage'] = _POINTCLOUDMESSAGE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

PointCloudMessage = _reflection.GeneratedProtocolMessageType('PointCloudMessage', (_message.Message,), {
  'DESCRIPTOR' : _POINTCLOUDMESSAGE,
  '__module__' : 'PointCloudMessage_pb2'
  # @@protoc_insertion_point(class_scope:rspub_pb.PointCloudMessage)
  })
_sym_db.RegisterMessage(PointCloudMessage)


# @@protoc_insertion_point(module_scope)