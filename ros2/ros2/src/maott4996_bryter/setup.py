from setuptools import find_packages, setup

package_name = 'maott4996_bryter'

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
    maintainer='hal',
    maintainer_email='hal@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'brytertjeneste = maott4996_bryter.brytertjeneste:main',
            'brytermonitor = maott4996_bryter.brytermonitor:main'
        ],
    },
)
