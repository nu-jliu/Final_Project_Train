from setuptools import find_packages, setup

package_name = "arm_control"

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
    maintainer="jingkun",
    maintainer_email="jingkunliu2025@u.northwestern.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"servo_control = {package_name}.servo_control:main",
            f"command_publisher = {package_name}.js_publisher:main",
            f"arm_teleop = {package_name}.arm_teleop:main",
        ],
    },
)
