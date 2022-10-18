from setuptools import setup

package_name = 'soccer_computer_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hvakil',
    maintainer_email='hvakil@olin.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_tracker = soccer_computer_vision.ball_tracker:main',
            'ball_tracker_solution = soccer_computer_vision.ball_tracker_solution:main',
            'ball_hitter = soccer_computer_vision.ball_hitter:main'
        ],
    },
)
