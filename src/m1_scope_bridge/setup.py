from setuptools import setup


package_name = "m1_scope_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Rosmaster user",
    maintainer_email="user@example.com",
    description="Offline rosbag export and SCOPE OGM preprocessing for Rosmaster M1.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "bag_export = m1_scope_bridge.bag_export:main",
        ],
    },
)
