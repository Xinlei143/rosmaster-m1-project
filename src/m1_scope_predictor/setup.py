import os

from setuptools import find_packages, setup


package_name = "m1_scope_predictor"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    package_data={
        package_name: ["third_party/templerail_scope/*"],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         ["launch/scope_online.launch.py"]),
        (os.path.join("share", package_name, "config"),
         ["config/scope_online.yaml"]),
        (os.path.join("share", package_name, "rviz"),
         ["rviz/scope_online.rviz"]),
    ],
    install_requires=["setuptools", "numpy"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Rosmaster user",
    maintainer_email="user@example.com",
    description="Observer-only online SCOPE prediction for the Rosmaster M1.",
    license="Apache-2.0 AND MIT",
    entry_points={
        "console_scripts": [
            "scope_predictor = m1_scope_predictor.predictor_node:main",
            "scope_runtime_parity = m1_scope_predictor.parity:main",
        ],
    },
)
