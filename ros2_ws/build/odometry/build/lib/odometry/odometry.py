import rclpy 
from rclpy.node import Node 
from std_msgs.msg import Float32
import math

class odometry(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.msgx = Float32()
        self.msgy = Float32()
        self.msgz = Float32()
        

        # Suscripciones a topicos establecidos
        self.encoderl_subscription = self.create_subscription(Float32, '/VelocityEncL', self.velocidadl_callback, rclpy.qos.qos_profile_sensor_data)
        self.encoderr_subscription = self.create_subscription(Float32, '/VelocityEncR', self.velocidadr_callback, rclpy.qos.qos_profile_sensor_data)

        # PUBlicadores de datos obtenicos
        self.pos_x_publisher =  self.create_publisher(Float32, 'PosicionX', rclpy.qos.qos_profile_sensor_data)
        self.pos_y_publisher =  self.create_publisher(Float32, 'PosicionY', rclpy.qos.qos_profile_sensor_data)
        self.orientacion_publisher =  self.create_publisher(Float32, 'Orientacion', rclpy.qos.qos_profile_sensor_data)


        # Variables donde se guardan los valores obtenidos de las suscripciones establecidas
        self.recep_encL = 0.0
        self.recep_encR = 0.0
        self.v_lineal = 0.0
        self.v_angular = 0.0
        self.Rueda = 0.05  
        self.Ldistancia = 0.18  
        self.theta = 0.0
        self.venx = 0.0
        self.veny = 0.0
        self.p_x = 0.3
        self.p_y = 2.4
  
  
  
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.total_timer_callback)

        self.get_logger().info("NODO DE ODOMETRIA..... ")

    def velocidadl_callback(self, msg):
        self.recep_encL = msg.data

    def velocidadr_callback(self, msg):  
        self.recep_encR = msg.data
                
    def total_timer_callback(self):
        self.v_lineal = self.Rueda*((self.recep_encL+self.recep_encR)/2.0)
        self.v_angular = self.Rueda*((self.recep_encR-self.recep_encL)/self.Ldistancia)
        self.theta = self.v_angular*self.timer_period + self.theta
        #distanciatrayecto_msg.data = self-v_lineal * self.timer_period
        #orientacion = (self.theta_orientacion + velang_msg.data * self.timer_period) % (2 * math.pi) # va de 0 a 2pi

        self.venx = self.v_lineal * math.cos(self.theta)
        self.veny = self.v_lineal * math.sin(self.theta)
        
        self.p_x = self.venx*self.timer_period + self.p_x
        self.p_y = self.veny*self.timer_period + self.p_y
        
        self.msgx.data = self.p_x
        self.msgy.data = self.p_y
        self.msgz.data = self.theta 
        self.pos_x_publisher.publish(self.msgx)
        self.pos_y_publisher.publish(self.msgy)
        self.orientacion_publisher.publish(self.msgz)
	        
        self.get_logger().info(f'Pos en X: {self.p_x:.2f} Pos en Y: {self.p_y:.2f} Angulo: {self.theta:.2f}')


        #self.get_logger().info(f'Dist {distanciatrayecto_msg.data} theta: {orientaciontotal_msg.data} Vel_Angular: {angulartotal_msg.data} X: {posicion_enX_msg.data} Y: {posicion_enY_msg.data}')  

def main(args=None):
    rclpy.init(args=args)
    process = odometry()
    rclpy.spin(process)
    process.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
