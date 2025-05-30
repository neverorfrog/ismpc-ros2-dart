import sys, os
webots_home = os.environ.get('WEBOTS_HOME')
controller_path = os.path.join(webots_home, 'lib', 'controller', 'python')
sys.path.append(controller_path)
from controller import Robot, AnsiCodes, Supervisor  
from geometry_msgs.msg import Pose
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
import rclpy
import os

from webots_lola_controller.controllers.nao_lola_supervisor.nao import Nao
import webots_lola_controller.controllers.nao_lola_supervisor.umsgpack as umsgpack
import socket

class NaoSupervisor(Node):
    def __init__(self):
        super().__init__('nao_supervisor')
        self.nao = Nao(self.get_logger())
        umsgpack.compatibility = True # this is needed to correctly receive the data from the LoLA client
        
        self.get_logger().info("Starting nao_supervisor")
        
        try:
            os.unlink(self.nao.SOCK_PATH)
        except OSError:
            if os.path.exists(self.nao.SOCK_PATH):
                raise

        if self.nao.args.tcp:
            sock_type = socket.AF_INET
        else:
            sock_type = socket.AF_UNIX
            

        self.sock = socket.socket(sock_type, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(self.nao.args.lola_addr)
        self.sock.listen()
        self.sock.settimeout((self.nao.timeStep-1)/1000.0)

        self.nao.print_status()

        self.conn = None
        self.imgCounter = self.nao.frametime
        
        self.timer = self.create_timer(self.nao.timeStep/1000.0, self.update)
        self.publisher = self.create_publisher(Pose, 'nao_pose', 10)
        
        self.get_logger().info("Nao supervisor initialized")
    
    def update(self):
        if self.nao.stepBegin(self.nao.timeStep) == -1:
            self.terminate()
            return
        
        self.key = self.nao.keyboard.getKey()
        if self.conn:
            self.nao.updateSensors()
            try:
                # send sensor data to LoLa client
                packed = umsgpack.packb(self.nao.sensors, force_float_precision="single")
                if len(packed) != self.nao.MSGPACK_READ_LENGTH:
                    self.get_logger().info(AnsiCodes.RED_FOREGROUND + "Msgpack packet size doesn't match LoLA specifications."  + AnsiCodes.RESET)
                self.conn.send(packed)
                
                # receive actuator data from LoLa client
                data = self.conn.recv(self.nao.ACTUATOR_PKT_SIZE*3)
                if data:
                    self.nao.updateActuators(umsgpack.unpackb(data))
            except TimeoutError:
                self.get_logger().info(AnsiCodes.RED_FOREGROUND + "Timeout while waiting for LoLa actuators." + AnsiCodes.RESET)
            except ConnectionError:
                self.conn.close()
                self.conn = None
                self.get_logger().info(AnsiCodes.RED_FOREGROUND + "LoLa client disconnected." + AnsiCodes.RESET)
        else:
            try:
                (self.conn, addr) = self.sock.accept()
                packed = umsgpack.packb(self.nao.sensors, force_float_precision="single")
                if len(packed) != self.nao.MSGPACK_READ_LENGTH:
                    self.get_logger().info(AnsiCodes.RED_FOREGROUND + "Msgpack packet size doesn't match LoLA specifications."  + AnsiCodes.RESET)
                self.get_logger().info("Sending sensor data to LoLa client")
                self.conn.send(packed)
                self.get_logger().info(AnsiCodes.GREEN_FOREGROUND + "LoLa client connected." + AnsiCodes.RESET)
                self.get_logger().info("OPENING CONNECTION")
            except:
                self.conn = None
                self.nao.stepEnd()
                return

        if self.nao.stepEnd() == -1:
            self.terminate()
            return
        
        # Publish the pose of the Nao (torso frame)
        msg = Pose()
        position = self.nao.tf.getSFVec3f()
        msg.position.x = position[0]
        msg.position.y = position[1]
        msg.position.z = position[2]
        
        orientation = self.nao.tf.getSFRotation()
        orientation = quaternion_from_euler(orientation[0], orientation[1], orientation[2])
        msg.orientation.x = orientation[0]
        msg.orientation.y = orientation[1]
        msg.orientation.z = orientation[2]
        msg.orientation.w = orientation[3]
        self.publisher.publish(msg)
    
    def terminate(self):
        self.sock.close()
        self.get_logger().info("Terminating nao_supervisor")
        self.destroy_node()
        
        # remove socket on exit
        os.unlink(self.nao.SOCK_PATH)

        # stop image threads
        if self.nao.args.camera:
            self.nao.topImageServer.stop()
            self.nao.bottomImageServer.stop()
            
        rclpy.shutdown()
        
def main(args=None):
    rclpy.init(args=args)
    nao_supervisor = NaoSupervisor()
    try:
        rclpy.spin(nao_supervisor)
    except Exception as e:
        nao_supervisor.get_logger().error(f"Error in main loop: {e}")
        nao_supervisor.terminate()
    finally:
        nao_supervisor.terminate()

if __name__ == "__main__":
    main()