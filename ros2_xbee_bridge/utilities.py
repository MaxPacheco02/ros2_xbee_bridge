import roslibpy
import rosidl_runtime_py


def msg_to_dict(msg):
    return roslibpy.Message(rosidl_runtime_py.convert.message_to_ordereddict(msg))


def dict_to_msg(msg_type, msg_dict):
    new_msg = msg_type()
    rosidl_runtime_py.set_message.set_message_fields(new_msg, msg_dict)
    return new_msg


def import_msg(msg_type):
    dir, msg = msg_type.split('/')
    pkg = __import__(f'{dir}.msg')
    return getattr(getattr(pkg, 'msg'), msg)