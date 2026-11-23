from __future__ import annotations

import os
import socket
import subprocess
from pathlib import Path
from typing import List, Tuple

CACHE_DIR = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "whatslab" / "webxr"
CERT_DAYS = 3650


def preferred_ip() -> str:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        if sock.connect_ex(("1.1.1.1", 53)) != 0:
            return "127.0.0.1"
        return sock.getsockname()[0]


def _san(ips: List[str]) -> str:
    entries = ["DNS:localhost", "IP:127.0.0.1", "IP:::1"]
    entries += [f"IP:{ip}" for ip in ips]
    return ",".join(entries)


def ensure_cert(ips: List[str] = None, cache_dir: Path = None) -> Tuple[str, str]:
    if ips is None:
        ip = preferred_ip()
        ips = [] if ip.startswith("127.") else [ip]
    cache = CACHE_DIR if cache_dir is None else Path(cache_dir)
    cache.mkdir(parents=True, exist_ok=True)
    cert = cache / "cert.pem"
    key = cache / "key.pem"
    san = _san(ips)
    stamp = cache / "san.txt"

    if cert.exists() and key.exists() and stamp.exists() and stamp.read_text() == san:
        return str(cert), str(key)

    cmd = ["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes",
           "-keyout", str(key), "-out", str(cert),
           "-days", str(CERT_DAYS), "-subj", "/CN=whatslab-webxr",
           "-addext", f"subjectAltName={san}"]
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=60.0)
    if res.returncode != 0:
        raise RuntimeError(f"인증서 생성 실패: {res.stderr.strip()}")
    key.chmod(0o600)
    stamp.write_text(san)
    return str(cert), str(key)
