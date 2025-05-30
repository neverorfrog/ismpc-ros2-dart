from setuptools import find_packages, setup

package_name = 'webots_lola_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(".", exclude=["test"]),
    package_dir={
        "": "."
    },  # Tell setuptools that packages are under current dir
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neverorfrog',
    maintainer_email='97flavio.maiorana@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "webots_lola_controller_exec = webots_lola_controller.controllers.nao_lola_supervisor.nao_lola_supervisor:main",
        ],
    },
)
