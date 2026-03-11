from setuptools import find_packages, setup

package_name = 'exo_test'

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
    maintainer='Muiz Hamzat',
    maintainer_email='muizhamzat@gmail.com',
    description='Testing sending custom messages of joint commands',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		    "test_publisher = exo_test.test_publisher:main",
            "test_subscriber = exo_test.test_subscriber:main",
        ],
    },
)
