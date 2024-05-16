from setuptools import find_packages, setup

package_name = "pick_and_place"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    py_modules=[package_name + ".MaxArm_ctl"],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="allen",
    maintainer_email="jingkunliu2025@u.northwestern.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
