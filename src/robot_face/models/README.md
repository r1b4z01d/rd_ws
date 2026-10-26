# Face models

Both files come from the [OpenCV model zoo](https://github.com/opencv/opencv_zoo) (Apache-2.0) and are
loaded by `robot_face/face_id.py` through OpenCV's built-in `FaceDetectorYN` / `FaceRecognizerSF`.

| File | Purpose | Size |
|---|---|---|
| `face_detection_yunet_2023mar.onnx` | YuNet face detector (needs OpenCV ≥ 4.8) | 0.2 MB |
| `face_recognition_sface_2021dec.onnx` | SFace 128-d face embedding | 37 MB |

They are installed to `share/robot_face/models` by `setup.py`.
