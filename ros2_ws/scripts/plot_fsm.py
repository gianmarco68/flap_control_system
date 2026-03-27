import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Float32, Float64
import matplotlib.pyplot as plt
import math

# Funzione di utilità per convertire i quaternioni in angoli di Eulero (Rollio, Beccheggio)
def get_euler_from_imu(msg):
    x, y, z, w = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(t0, t1))
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.degrees(math.asin(t2))
    return roll, pitch

class FsmPlotterNode(Node):
    def __init__(self):
        # IL SEGRETO: Diciamo a ROS di usare il tempo della rosbag!
        super().__init__('fsm_plotter_node', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])
        
        # CREA IL PROFILO QOS UNIVERSALE
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Accetta dati anche se non garantiti
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.start_time = None
        
        # ===================================================================
        # CONFIGURAZIONE TOPIC 
        # Aggiungi qui i topic della bag (sensori) e quelli del tuo PID.
        # Formato: ('NomeTopic', TipoMessaggio, {'Etichetta in Legenda': lambda msg: estrazione_dato})
        # ===================================================================
        self.topics_config = [
            ('/fix_velocity', TwistWithCovarianceStamped, {
                'Velocità (Nodi)': lambda msg: math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2) * 1.94384
            }),
            ('/imu_data', Imu, {
                'Rollio (deg)': lambda msg: get_euler_from_imu(msg)[0],
                'Beccheggio (deg)': lambda msg: get_euler_from_imu(msg)[1]
            }),
            ('/wand_angle', Float32, {
                'Angolo Wand (deg)': lambda msg: msg.data
            }),
            # PRESUMIAMO CHE QUESTO SIA IL TOPIC IN USCITA DAL TUO NODO PID:
            ('/control_value', Float64, {
                'Comando Flap dal PID (deg)': lambda msg: msg.data
            })
        ]
        
        # Struttura dati dinamica che conterrà tempi e valori indipendenti per ogni variabile
        self.data = {} 
        
        # Setup automatico di tutte le callback e le iscrizioni
        for topic_name, msg_type, extractors in self.topics_config:
            for label in extractors.keys():
                self.data[label] = {'times': [], 'values': []}
            
            cb = self.create_callback(extractors)
            self.create_subscription(msg_type, topic_name, cb, qos_profile)

        self.get_logger().info("📊 Plotter avviato in ascolto con QoS Best Effort! Lancia la bag e il tuo PID.")
        self.get_logger().info("👉 Premi Ctrl+C in questo terminale alla fine per generare il grafico.")

    def create_callback(self, extractors):
        """Fabbrica di funzioni: crea una callback su misura per ogni topic"""
        def callback(msg):
            # Leggiamo il tempo esatto in cui arriva questo specifico messaggio
            now = self.get_clock().now().nanoseconds / 1e9
            
            if self.start_time is None:
                self.start_time = now # Fissiamo il "Tempo Zero" al primo messaggio ricevuto
                
            t = now - self.start_time
            
            # Estraiamo i dati richiesti e li salviamo nelle loro liste dedicate
            for label, extract_func in extractors.items():
                # Evita problemi di reset se la bag ricomincia da capo
                if t < 0 or (len(self.data[label]['times']) > 0 and t < self.data[label]['times'][-1]):
                    continue
                
                try:
                    val = extract_func(msg)
                    self.data[label]['times'].append(t)
                    self.data[label]['values'].append(val)
                except Exception as e:
                    self.get_logger().error(f"Errore nell'estrazione di {label}: {e}")
                    
        return callback

def main(args=None):
    rclpy.init(args=args)
    node = FsmPlotterNode()
    
    try:
        rclpy.spin(node) # Mantieni il nodo in ascolto
    except KeyboardInterrupt:
        # Quando premi Ctrl+C, interrompi l'ascolto e passa al disegno
        node.get_logger().info("Interruzione ricevuta. Generazione grafico in corso...")
    finally:
        # Controlliamo che ci sia almeno un dato registrato
        has_data = any(len(d['times']) > 0 for d in node.data.values())
        
        if has_data:
            plt.figure(figsize=(12, 8))
            
            # Disegna tutte le variabili sullo stesso asse temporale
            for label, series in node.data.items():
                if len(series['times']) > 0:
                    plt.plot(series['times'], series['values'], label=label, linewidth=2)
            
            plt.xlabel('Tempo (Simulazione) [s]')
            plt.ylabel('Valori (Unità miste)')
            plt.title('Test PID su Dati Reali della Rosbag')
            plt.legend()
            plt.grid(True)
            
            plt.tight_layout()
            plt.savefig('test_pid_coordinato.png')
            print("✅ Grafico salvato con successo in 'test_pid_coordinato.png'!")
        else:
            print("⚠️ Nessun dato ricevuto. Assicurati che la bag e il nodo PID stiano pubblicando sui topic corretti.")
            
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()