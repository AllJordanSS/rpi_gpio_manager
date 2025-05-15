#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import RPi.GPIO as GPIO
import os
import yaml
from ament_index_python.packages import get_package_share_directory


class GPIOManager(Node):
    def __init__(self):
        super().__init__('gpio_manager')
        self.get_logger().info("GPIO Manager iniciado...")

        # Carregar parâmetros do YAML
        self.config_path = os.path.join(
            get_package_share_directory('gpio_manager'),
            'config',
            'params.yaml'
        )

        try:
            with open(self.config_path, 'r') as f:
                self.params = yaml.safe_load(f)['gpio_config']
        except Exception as e:
            self.get_logger().error(f"Erro ao carregar params.yaml: {e}")
            raise

        self.pin_motor_1 = self.params['pin_motor_1']
        self.pin_motor_2 = self.params['pin_motor_2']
        self.service_name = self.params['service_name']

        # Carregar mensagens com fallback seguro
        self.messages = self.params.get('messages', {})
        self.get_logger().info(f"Mensagens carregadas: {self.messages}")

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_motor_1, GPIO.OUT)
        GPIO.setup(self.pin_motor_2, GPIO.OUT)

        # Estado inicial: motor desligado
        GPIO.output(self.pin_motor_1, GPIO.LOW)
        GPIO.output(self.pin_motor_2, GPIO.HIGH)

        # Serviço ROS 2
        self.srv = self.create_service(
            SetBool,
            self.service_name,
            self.motor_callback
        )
        self.get_logger().info(f"Serviço '{self.service_name}' pronto.")

    def motor_callback(self, request, response):
        # Verificar se o campo 'data' é True ou False
        if not isinstance(request.data, bool):
            response.success = False
            response.message = (
                "Tipo inválido. O campo 'data' deve ser um booleano (true ou false). "
                "Exemplo válido: {\"data\": true}"
            )
            self.get_logger().warn("Recebido valor inválido para 'data'.")
            return response

        # Processar solicitação
        if request.data:
            self.get_logger().info("Ativando motores...")
            GPIO.output(self.pin_motor_1, GPIO.HIGH)
            GPIO.output(self.pin_motor_2, GPIO.LOW)
            response.success = True
            response.message = self.messages.get('true', "Motores ligados.")
        else:
            self.get_logger().info("Desligando motores...")
            GPIO.output(self.pin_motor_1, GPIO.LOW)
            GPIO.output(self.pin_motor_2, GPIO.HIGH)
            response.success = True
            response.message = self.messages.get('false', "Motores desligados.")

        return response

    def destroy_node(self):
        GPIO.cleanup()
        self.get_logger().info("GPIOs limpos.")
        super().destroy_node()


def main():
    rclpy.init()
    node = GPIOManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()