import rclpy.serialization
from std_msgs.msg import Bool, String


def serialize_message(msg, use_ros=False):
    """
    Serialize a message object.

    Use a custom serialization method if available. Otherwise use built-in
    ROS serialization. Writing custom serialization methods can help reduce
    the serialized message size.

    Args:
        msg: The message object to be serialized.
        use_ros: Flag indicating whether to force ROS serialization.

    Returns:
        The serialized message as bytes.
    """
    if use_ros:
        return rclpy.serialization.serialize_message(msg)

    if isinstance(msg, String):
        return bytes(msg.data, "ascii")
    if isinstance(msg, Bool):
        return bytes([msg.data])
    return rclpy.serialization.serialize_message(msg)


def deserialize_message(data, msg_type, use_ros=False):
    """
    Deserialize the given data into a ROS message of the specified type.

    Use a custom deserialization method if available. Otherwise use built-in
    ROS deserialization. Using custom deserialization methods is mandatory
    when using the matching custom serialization methods.

    Args:
        data: The serialized data to be deserialized.
        msg_type: The type of the ROS message to be deserialized into.
        use_ros: Flag indicating whether to force ROS deserialization.

    Returns:
        The deserialized ROS message.
    """
    if use_ros:
        return rclpy.serialization.deserialize_message(data, msg_type)

    if msg_type == String:
        return String(data=data.decode("ascii"))
    if msg_type == Bool:
        return Bool(data=bool(data[0]))
    return rclpy.serialization.deserialize_message(bytes(data), msg_type)
