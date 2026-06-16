from setuptools import find_packages, setup

package_name = 'enwbot_helper_nodes'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mcbed',
    maintainer_email='mcbed@todo.todo',
    description='Helper nodes for enwbot GUI control.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'enwbot_gui_node = enwbot_helper_nodes.enwbot_gui_node:main',
        ],
    },
)
