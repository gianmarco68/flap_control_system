import time
import math
import struct
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import serialize_message
from rclpy.time import Time

# Tipi di messaggio
from sail_msgs.msg import SerialMsg
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped

def get_time_obj(ns):
    t = Time(nanoseconds=ns)
    return t.to_msg()

def pack_le16(value_float, scale=1.0):
    val_int = int(value_float / scale)
    val_int = max(min(val_int, 32767), -32768)
    return list(struct.pack('<h', val_int))

def create_bag():
    bag_name = 'rosbag2_milan_simulation'
    print(f"🚀 Generazione Rosbag Dinamica: {bag_name}...")

    writer = SequentialWriter()
    storage_options = StorageOptions(uri=bag_name, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    writer.open(storage_options, converter_options)

    # === DEFINIZIONE TOPIC ===
    topics = {
        '/serial/raw_rx': 'sail_msgs/msg/SerialMsg',
        '/fix': 'sensor_msgs/msg/NavSatFix',
        '/fix_velocity': 'geometry_msgs/msg/TwistWithCovarianceStamped'
    }

    for i, (name, type_name) in enumerate(topics.items()):
        topic_info = TopicMetadata(
            id=(i + 1),
            name=name,
            type=type_name,
            serialization_format='cdr'
        )
        writer.create_topic(topic_info)

    # === PARAMETRI SIMULAZIONE ===
    duration_sec = 600
    freq_serial = 100 
    freq_gps = 5      
    
    start_ns = time.time_ns()
    
    # Centro di Milano
    lat0 = 45.4642
    lon0 = 9.1900
    radius = 100 
    
    steps = duration_sec * freq_serial
    
    for i in range(steps):
        t_now = start_ns + int(i * (1e9 / freq_serial))
        time_msg = get_time_obj(t_now)
        elapsed = i / freq_serial

        # === FISICA DINAMICA ===
        # La velocità oscilla tra 3 e 7 m/s
        current_speed = 5.0 + 2.0 * math.sin(elapsed * 0.2)

        angle_rad = (elapsed * current_speed / radius)
        
        d_lat = (radius * math.cos(angle_rad)) / 111320.0
        d_lon = (radius * math.sin(angle_rad)) / (40075000.0 * math.cos(lat0 * math.pi / 180.0) / 360.0)
        curr_lat = lat0 + d_lat
        curr_lon = lon0 + d_lon

        roll_deg = (5.0 + current_speed) * math.sin(elapsed * 2.0)
        pitch_deg = 5.0 * math.cos(elapsed * 1.5)
        yaw_deg = (math.degrees(angle_rad) + 90) % 360 

        wand_deg = 45.0 * math.sin(elapsed * 0.5)

        # 1. SERIAL MSG (IMU + WAND)
        seq = i % 256 
        
        # ID 26: WAND
        msg_wand = SerialMsg()
        msg_wand.stamp = time_msg 
        msg_wand.id = 26
        msg_wand.payload.data = [seq] + pack_le16(wand_deg, 1.0) 
        writer.write('/serial/raw_rx', serialize_message(msg_wand), t_now)

        # ID 145: IMU RPY
        msg_rpy = SerialMsg()
        msg_rpy.stamp = time_msg
        msg_rpy.id = 145
        msg_rpy.payload.data = [seq] + \
                               pack_le16(yaw_deg, 0.01) + \
                               pack_le16(pitch_deg, 0.01) + \
                               pack_le16(roll_deg, 0.01)
        writer.write('/serial/raw_rx', serialize_message(msg_rpy), t_now)

        # ID 57: IMU VEL
        msg_vel = SerialMsg()
        msg_vel.stamp = time_msg
        msg_vel.id = 57
        msg_vel.payload.data = [seq] + pack_le16(0, 0.01) + pack_le16(0, 0.01) + pack_le16(0, 0.01)
        writer.write('/serial/raw_rx', serialize_message(msg_vel), t_now)

        # ID 41: IMU ACC
        msg_acc = SerialMsg()
        msg_acc.stamp = time_msg
        msg_acc.id = 41
        msg_acc.payload.data = [seq] + pack_le16(0, 0.01) + pack_le16(0, 0.01) + pack_le16(9.81, 0.01)
        writer.write('/serial/raw_rx', serialize_message(msg_acc), t_now)

        # 2. GPS DATA
        if i % (freq_serial // freq_gps) == 0:
            # /fix
            gps_fix = NavSatFix()
            gps_fix.header.stamp = time_msg
            gps_fix.header.frame_id = "gps"
            gps_fix.latitude = curr_lat
            gps_fix.longitude = curr_lon
            gps_fix.altitude = 120.0
            gps_fix.status.status = 0
            writer.write('/fix', serialize_message(gps_fix), t_now)

            # /fix_velocity (CON LA SOG DINAMICA!)
            gps_vel = TwistWithCovarianceStamped()
            gps_vel.header.stamp = time_msg
            gps_vel.header.frame_id = "gps"
            gps_vel.twist.twist.linear.x = current_speed # <--- Qui cambia!
            writer.write('/fix_velocity', serialize_message(gps_vel), t_now)

    print(f"✅ Fatto! Bag salvata in: {bag_name}")

if __name__ == '__main__':
    create_bag()