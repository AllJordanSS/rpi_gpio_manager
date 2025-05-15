from setuptools import setup

package_name = 'gpio_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools', 'rclpy', 'std_srvs', 'PyYAML'],
    zip_safe=True,
    maintainer='AllJordanSS',
    maintainer_email='xulipasouza@hotmail.com',
    description='Pacote ROS 2 para gerenciar GPIOs via serviço',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'gpio_node = gpio_manager.gpio_node:main',
        ],
    },
)