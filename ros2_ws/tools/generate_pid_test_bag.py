import time
import math
import struct
import random
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

def create_pid_test_bag():
    bag_name = 'pid_test_bag'
    print(f"🚀 Generazione Rosbag di Test per PID: {bag_name}...")

    writer = SequentialWriter()
    storage_options = StorageOptions(uri=bag_name, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    writer.open(storage_options, converter_options)

    # === DEFINIZIONE TOPIC ===
    topics = {
        '/serial/raw_rx': 'sail_msgs/msg/SerialMsg',
        '/gps_data': 'sensor_msgs/msg/NavSatFix',
        '/gps_vel_data': 'geometry_msgs/msg/TwistWithCovarianceStamped'
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
    duration_sec = 240 # 4 fasi da 20 secondi l'una
    freq_serial = 100 
    freq_gps = 5      
    
    start_ns = time.time_ns()
    
    # Centro di Milano (Base di partenza)
    lat0 = 45.4642
    lon0 = 9.1900
    
    steps = duration_sec * freq_serial
    
    # Variabili di stato (per accumulare posizione in modo sensato)
    curr_lat = lat0
    curr_lon = lon0
    
    for i in range(steps):
        t_now = start_ns + int(i * (1e9 / freq_serial))
        time_msg = get_time_obj(t_now)
        elapsed = i / freq_serial

        # ==========================================
        # === MACCHINA A STATI DELLA SIMULAZIONE ===
        # ==========================================
        
        # Valori di base (vengono sovrascritti dalle fasi)
        current_speed = 5.0
        roll_deg = 0.0
        pitch_deg = 0.0
        yaw_deg = 45.0 # Rotta costante
        wand_deg = 30.0
        
        # FASE 1: Navigazione Normale (0 - 20s)
        if elapsed < 60:
            roll_deg = 2.0 * math.sin(elapsed)
            pitch_deg = 1.0 * math.cos(elapsed * 0.5)
            wand_deg = 30.0 + random.uniform(-0.5, 0.5) # Rumore trascurabile
            
        # FASE 2: SCUFFIA! (20 - 40s)
        elif elapsed < 120:
            # Transizione rapida a 90 gradi di roll
            roll_deg = 90.0 + random.uniform(-2.0, 2.0)
            pitch_deg = 0.0
            current_speed = 0.5 # Barca ferma scuffiata
            wand_deg = 0.0 # Wand fuori dall'acqua o piegata
            
        # FASE 3: VOLO IN PICCHIATA! (40 - 60s)
        elif elapsed < 180:
            roll_deg = 5.0 * random.uniform(-1, 1) # Molto instabile
            # Transizione brutale del pitch (prua in giù)
            pitch_deg = -45.0 + random.uniform(-3.0, 3.0) 
            current_speed = max(0.0, 10.0 - (elapsed - 40) * 2) # Frena di botto da 10m/s a 0
            wand_deg = 45.0 + random.uniform(-5.0, 5.0) # Wand schiacciata in acqua
            
        # FASE 4: Wand Impazzita / Guasto Sensore (60 - 80s)
        else:
            roll_deg = 5.0 * math.sin(elapsed * 2)
            pitch_deg = 2.0 * math.cos(elapsed)
            current_speed = 6.0
            # Rumore pazzesco sulla wand (media 30, ma sbalzi enormi)
            wand_deg = 30.0 + random.uniform(-25.0, 25.0) 

        # === AGGIORNAMENTO POSIZIONE GPS SEMPLIFICATA ===
        # Calcoliamo lo spostamento in base alla rotta (yaw) e velocità
        d_lat = (current_speed * math.cos(math.radians(yaw_deg)) * (1.0/freq_serial)) / 111320.0
        d_lon = (current_speed * math.sin(math.radians(yaw_deg)) * (1.0/freq_serial)) / (40075000.0 * math.cos(math.radians(lat0)) / 360.0)
        curr_lat += d_lat
        curr_lon += d_lon

        # ==========================================
        # === CREAZIONE E SCRITTURA MESSAGGI ===
        # ==========================================
        
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
            # /gps_data
            gps_fix = NavSatFix()
            gps_fix.header.stamp = time_msg
            gps_fix.header.frame_id = "gps"
            gps_fix.latitude = curr_lat
            gps_fix.longitude = curr_lon
            gps_fix.altitude = 120.0
            gps_fix.status.status = 0
            writer.write('/gps_data', serialize_message(gps_fix), t_now)

            # /gps_vel_data
            gps_vel = TwistWithCovarianceStamped()
            gps_vel.header.stamp = time_msg
            gps_vel.header.frame_id = "gps"
            gps_vel.twist.twist.linear.x = current_speed 
            writer.write('/gps_vel_data', serialize_message(gps_vel), t_now)

    print(f"✅ Fatto! Bag salvata in: {bag_name}")

if __name__ == '__main__':
    create_pid_test_bag()