from setuptools import find_packages, setup

package_name = "motor_testbench"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sat",
    maintainer_email="sat@todo.todo",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_test = motor_testbench.robot_test:main",
            "robot_read_passive_angles = motor_testbench.robot_read_passive_angles:main",
            "motor_test = motor_testbench.motor_test:main",
            "motor_message_passer = motor_testbench.motor_message_passer:main",
        ],
    },
)
