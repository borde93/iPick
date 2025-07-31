from setuptools import find_packages, setup

package_name = 'swiftpro_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='enrico',
    maintainer_email='enrico@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'read_status = swiftpro_py.swiftpro_read_node:main',
            'move_arm_server = swiftpro_py.swiftpro_move_server_node:main',
            'move_arm_client = swiftpro_py.swiftpro_move_client_node:main',

        ],
    },
)
