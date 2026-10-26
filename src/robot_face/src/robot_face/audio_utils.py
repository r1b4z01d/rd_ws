"""Small audio helpers for driving the mouth waveform — no ROS dependency."""

from __future__ import annotations

import wave
from typing import Iterator, Tuple

import numpy as np


def load_wav_mono(path: str) -> Tuple[np.ndarray, int]:
    """Read a PCM WAV file as mono float32 samples in [-1, 1].

    Returns ``(samples, sample_rate)``. Multi-channel files are averaged down
    to mono. 8-, 16- and 32-bit integer PCM are supported.
    """
    with wave.open(path, "rb") as wf:
        rate = wf.getframerate()
        channels = wf.getnchannels()
        width = wf.getsampwidth()
        raw = wf.readframes(wf.getnframes())

    if width == 2:
        audio = np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0
    elif width == 4:
        audio = np.frombuffer(raw, dtype="<i4").astype(np.float32) / 2147483648.0
    elif width == 1:
        audio = (np.frombuffer(raw, dtype=np.uint8).astype(np.float32) - 128.0) / 128.0
    else:
        raise ValueError(f"unsupported WAV sample width: {width} bytes")

    if channels > 1:
        usable = (audio.size // channels) * channels
        audio = audio[:usable].reshape(-1, channels).mean(axis=1)

    return np.ascontiguousarray(audio, dtype=np.float32), int(rate)


def write_wav_mono16(path: str, audio: np.ndarray, rate: int) -> None:
    """Write float samples in [-1, 1] as a 16-bit mono PCM WAV."""
    pcm = (np.clip(np.asarray(audio, dtype=np.float32), -1.0, 1.0) * 32767.0).astype("<i2")
    with wave.open(path, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(int(rate))
        wf.writeframes(pcm.tobytes())


def prepend_silence(audio: np.ndarray, rate: int, seconds: float,
                    level: float = 0.0, tone_hz: float = 50.0) -> np.ndarray:
    """Return ``audio`` with a ``seconds``-long lead-in in front.

    With ``level`` = 0 the lead-in is digital silence. Many Bluetooth speakers
    only wake up when they detect a real signal, so a small ``level`` (e.g.
    0.01 ≈ -40 dBFS) plays a faint low tone instead, faded in and out so it
    doesn't click. Either way the mouth stays still for this stretch because
    the samples are (near) silent.
    """
    n = int(max(seconds, 0.0) * rate)
    if n <= 0:
        return audio
    if level <= 0.0:
        lead = np.zeros(n, dtype=np.float32)
    else:
        t = np.arange(n, dtype=np.float32) / float(rate)
        ramp = np.minimum(1.0, np.minimum(t, (n - 1) / rate - t) / 0.1).clip(0.0, 1.0)   # 100 ms fades
        lead = (np.sin(2.0 * np.pi * tone_hz * t) * float(level) * ramp).astype(np.float32)
    return np.concatenate([lead, np.asarray(audio, dtype=np.float32)])


def iter_chunks(audio: np.ndarray, rate: int, chunk_seconds: float) -> Iterator[Tuple[np.ndarray, float]]:
    """Yield ``(chunk, end_time_seconds)`` slices of ``audio`` in playback order.

    ``end_time_seconds`` is when playback of that chunk finishes, measured from
    the start of the clip — useful for pacing a real-time feed.
    """
    chunk = max(1, int(rate * chunk_seconds))
    for start in range(0, audio.size, chunk):
        stop = min(start + chunk, audio.size)
        yield audio[start:stop], stop / float(rate)


def chunk_level(samples: np.ndarray, gain: float = 1.0) -> Tuple[np.ndarray, float]:
    """Scale/clip a chunk and compute a 0..1 loudness level from its RMS.

    A full-scale sine has RMS ≈ 0.71, so RMS is scaled by 2.5 to let normal
    speech reach ~1.0.
    """
    data = np.clip(np.asarray(samples, dtype=np.float32) * float(gain), -1.0, 1.0)
    if data.size == 0:
        return data, 0.0
    rms = float(np.sqrt(np.mean(data * data)))
    return data, min(rms * 2.5, 1.0)


__all__ = ["load_wav_mono", "write_wav_mono16", "prepend_silence", "iter_chunks", "chunk_level"]
