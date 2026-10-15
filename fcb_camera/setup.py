import glob

from setuptools import setup

package_name = "fcb_camera"

setup(
    name=package_name,
    version="0.1.0",
    # fcb_base_driver is the camera code from fcb_orin (fcb_view.py and the
    # flight recorder), vendored: the modules the node needs, nothing else.
    packages=[package_name, "fcb_base_driver"],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Vikraant Rathore",
    maintainer_email="vikraant.rathore@gmail.com",
    description="ROS 2 driver for the Sony FCB-EV9520L",
    license="MIT",
    entry_points={
        "console_scripts": [
            "camera_node = fcb_camera.camera_node:main",
        ],
    },
)
