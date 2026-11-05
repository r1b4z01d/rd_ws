#!/usr/bin/env python3
"""InsightFace (SCRFD detector + ArcFace embedding) worker for the robot face.

Runs in its own virtualenv (insightface/onnxruntime pins differ from the ROS
python), so the face node streams frames to it over a pipe and gets faces plus
512-d embeddings back. Models: the ``buffalo_l`` pack (SCRFD-10GF + ArcFace
ResNet50 trained on WebFace600K), downloaded by insightface on first use into
``~/.insightface/models``.

Protocol (stdin → stdout):
    request : one JSON header line, then the raw frame bytes it announces
              {"w": 640, "h": 480, "c": 3, "max_faces": 3, "embed": true}\n<w*h*c bytes BGR uint8>
              ("embed": false = detection only, used for tracking at camera rate)
    reply   : {"ok": true, "ms": 87.1,
               "faces": [{"box": [x, y, w, h], "score": 0.93,
                          "kps": [[x, y] * 5], "emb": [512 floats, L2-normalised]}, ...]}
              {"ok": false, "error": "..."}
    header  : {"cmd": "quit"} exits; {"cmd": "info"} returns providers / det size.
On start the worker prints {"ok": true, "ready": true, "providers": [...], "dim": 512}.

Test by hand:
    faceid-venv/bin/python faceid_worker.py --image photo.jpg
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

EMBED_DIM = 512


def make_app(model_name: str, det_size: int, gpu: bool, max_faces: int, threads: int = 2):
    # insightface logs verbosely through print(); keep stdout clean for the protocol
    real_stdout = sys.stdout
    sys.stdout = sys.stderr
    try:
        import onnxruntime as ort
        from insightface.app import FaceAnalysis
        providers = ["CPUExecutionProvider"]
        if gpu and "CUDAExecutionProvider" in ort.get_available_providers():
            providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        # onnxruntime's default pool uses every core AND spin-waits between runs: on the
        # Orin that was five cores of continuous load. A couple of threads, no spinning.
        so = ort.SessionOptions()
        so.intra_op_num_threads = max(1, int(threads))
        so.inter_op_num_threads = 1
        so.add_session_config_entry("session.intra_op.allow_spinning", "0")
        so.add_session_config_entry("session.inter_op.allow_spinning", "0")
        # FaceAnalysis forwards extra kwargs to onnxruntime.InferenceSession
        app = FaceAnalysis(name=model_name, providers=providers, sess_options=so,
                           allowed_modules=["detection", "recognition"])
        app.prepare(ctx_id=0 if providers[0].startswith("CUDA") else -1,
                    det_size=(det_size, det_size))
        # warm up both models (CUDA kernels / allocations) so the first live frame is fast
        try:
            dummy = np.zeros((480, 640, 3), dtype=np.uint8)
            app.det_model.detect(dummy, max_num=1, metric="default")
            rec = app.models.get("recognition")
            if rec is not None:
                rec.get_feat(np.zeros((112, 112, 3), dtype=np.uint8))
        except Exception:
            pass
    finally:
        sys.stdout = real_stdout
    return app, providers


def analyze(app, frame: np.ndarray, max_faces: int, embed: bool = True) -> list:
    """Detect faces; attach ArcFace embeddings when ``embed`` is set.

    Tracking frames arrive at camera rate and only need boxes, so they skip the
    recognition model (the detector alone is ~6 ms on the Orin GPU).
    """
    from insightface.app.common import Face
    bboxes, kpss = app.det_model.detect(frame, max_num=max_faces, metric="default")
    out = []
    rec = app.models.get("recognition") if embed else None
    for i in range(bboxes.shape[0]):
        bbox = bboxes[i, 0:4]
        det_score = float(bboxes[i, 4])
        kps = kpss[i] if kpss is not None else None
        face = Face(bbox=bbox, kps=kps, det_score=det_score)
        if rec is not None:
            rec.get(frame, face)
        x0, y0, x1, y1 = [float(v) for v in bbox]
        rec_out = {
            "box": [int(round(x0)), int(round(y0)), int(round(x1 - x0)), int(round(y1 - y0))],
            "score": det_score,
            "kps": [[float(a), float(b)] for a, b in np.asarray(kps)] if kps is not None else [],
        }
        if rec is not None:
            rec_out["emb"] = [round(float(v), 5) for v in np.asarray(face.normed_embedding, dtype=np.float32)]
        out.append(rec_out)
    # largest first
    out.sort(key=lambda d: d["box"][2] * d["box"][3], reverse=True)
    return out


def serve(app, providers, max_faces_default: int) -> None:
    out = sys.stdout
    stdin = sys.stdin.buffer
    print(json.dumps({"ok": True, "ready": True, "providers": providers, "dim": EMBED_DIM}),
          file=out, flush=True)
    while True:
        header = stdin.readline()
        if not header:
            break
        header = header.strip()
        if not header:
            continue
        try:
            req = json.loads(header)
            cmd = req.get("cmd")
            if cmd == "quit":
                break
            if cmd == "info":
                reply = {"ok": True, "providers": providers, "dim": EMBED_DIM}
            else:
                w, h, c = int(req["w"]), int(req["h"]), int(req.get("c", 3))
                n = w * h * c
                buf = stdin.read(n)
                if len(buf) != n:
                    break   # stream truncated; the node will restart us
                frame = np.frombuffer(buf, dtype=np.uint8).reshape(h, w, c)
                t0 = time.perf_counter()
                faces = analyze(app, frame, int(req.get("max_faces", max_faces_default)),
                                embed=bool(req.get("embed", True)))
                reply = {"ok": True, "ms": round((time.perf_counter() - t0) * 1000.0, 1), "faces": faces}
        except Exception as e:   # never die on a bad request
            reply = {"ok": False, "error": f"{type(e).__name__}: {e}"}
        print(json.dumps(reply), file=out, flush=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default="buffalo_l", help="insightface model pack (buffalo_l, buffalo_s)")
    ap.add_argument("--det-size", type=int, default=480, help="detector input size (multiple of 32)")
    ap.add_argument("--gpu", action="store_true", help="use CUDAExecutionProvider when available")
    ap.add_argument("--max-faces", type=int, default=3)
    ap.add_argument("--threads", type=int, default=2, help="onnxruntime intra-op threads")
    ap.add_argument("--nice", type=int, default=5, help="process niceness (the face animation should win)")
    ap.add_argument("--image", help="analyse this image and print the result (test mode)")
    args = ap.parse_args()

    if args.nice:
        try:
            os.nice(args.nice)
        except OSError:
            pass
    t0 = time.perf_counter()
    app, providers = make_app(args.model, args.det_size, args.gpu, args.max_faces, args.threads)
    load_s = time.perf_counter() - t0

    if args.image:
        import cv2
        img = cv2.imread(args.image)
        if img is None:
            print(f"cannot read {args.image}", file=sys.stderr)
            return 1
        t0 = time.perf_counter()
        faces = analyze(app, img, args.max_faces)
        ms = (time.perf_counter() - t0) * 1000.0
        print(f"loaded in {load_s:.1f}s ({providers[0]}); {len(faces)} face(s) in {ms:.0f} ms")
        for f in faces:
            print(f"  box {f['box']} score {f['score']:.2f} emb[:4] {f['emb'][:4]}")
        return 0

    serve(app, providers, args.max_faces)
    return 0


if __name__ == "__main__":
    sys.exit(main())
