from setuptools import setup

package_name = "robot_face"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", ["launch/combined_face_hand.launch.py"]),
        ("share/" + package_name + "/config", ["config/combined_face_hand_defaults.yaml"]),
        (
            "share/" + package_name + "/models",
            [
                "models/face_detection_yunet_2023mar.onnx",
                "models/face_recognition_sface_2021dec.onnx",
                "models/README.md",
            ],
        ),
    ],
    # Runtime deps come from package.xml / system packages; listing pip names here makes
    # pkg_resources raise DistributionNotFound (e.g. inside TTS import) when OpenCV is
    # installed as python3-opencv instead of the pip "opencv-python" wheel.
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Bryan Ribas",
    maintainer_email="bryanribas@gmail.com",
    description="ROS 2 eyes + hand tracking node using MediaPipe and a subscribed image topic.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "combined_face_hand_node = robot_face.combined_face_hand_node:main",
        ],
    },
)
