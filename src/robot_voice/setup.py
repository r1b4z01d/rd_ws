from setuptools import setup

package_name = "robot_voice"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", ["config/voice_assistant.yaml"]),
        ("share/" + package_name + "/launch", ["launch/voice_assistant.launch.py"]),
    ],
    # Runtime deps come from package.xml / the voice venv; see provision/setup_voice.sh
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Bryan Ribas",
    maintainer_email="bryanribas@gmail.com",
    description="Voice assistant: wake phrase, Whisper ASR, Claude brain, skills.",
    license="CC BY-NC-SA",
    entry_points={
        "console_scripts": [
            "voice_assistant_node = robot_voice.voice_assistant_node:main",
        ],
    },
)
