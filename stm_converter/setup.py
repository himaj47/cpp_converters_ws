from setuptools import find_packages, setup

package_name = 'stm_converter'

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
    package_data={
        "stm_converter": [
            "resources/jinja_templates/*.txt",
        ],
    },
    zip_safe=True,
    author='himaj joshi',
    author_email='himajjoshi932@gmail.com',
    maintainer='himaj joshi',
    classifiers=[
        'Intended Audience :: Developers',
        'License :: TODO',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ROS2 package for converting c/c++ structs to ROS messages',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "stm_converter = stm_converter.main:main"
        ],
    },
)
