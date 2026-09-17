from __future__ import annotations

import ipaddress
import os
import socket
import subprocess
from pathlib import Path
from typing import List, Tuple

CACHE_DIR = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "whatslab" / "webxr"
CERT_DAYS = 3650


def local_ips() -> List[str]:
    out: List[str] = []
    try:
        res = subprocess.run(["ip", "-4", "-o", "addr", "show", "scope", "global"],
                             capture_output=True, text=True, timeout=5.0)
        for line in res.stdout.splitlines():
            parts = line.split()
            if len(parts) > 3 and parts[2] == "inet":
                out.append(parts[3].split("/")[0])
    except (OSError, subprocess.SubprocessError):
        pass
    if not out:
        try:
            out.append(socket.gethostbyname(socket.gethostname()))
        except OSError:
            pass
    return [ip for ip in out if not ip.startswith("127.")]


def route_ip() -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("1.1.1.1", 53))
        return sock.getsockname()[0]
    except OSError:
        return ""
    finally:
        sock.close()


def preferred_ip() -> str:
    ips = local_ips()
    routed = route_ip()
    if routed and not routed.startswith("127."):
        return routed
    for net in ("192.168.0.0/16", "10.0.0.0/8", "172.16.0.0/12"):
        block = ipaddress.ip_network(net)
        hit = [ip for ip in ips if ipaddress.ip_address(ip) in block]
        if hit:
            return hit[0]
    return ips[0] if ips else "127.0.0.1"


def _san(ips: List[str]) -> str:
    entries = ["DNS:localhost", "IP:127.0.0.1", "IP:::1"]
    entries += [f"IP:{ip}" for ip in ips]
    return ",".join(entries)


def ensure_cert(ips: List[str] = None, cache_dir: Path = None) -> Tuple[str, str]:
    ips = local_ips() if ips is None else list(ips)
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
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=60.0)
    except FileNotFoundError as exc:
        raise RuntimeError(
            "openssl 실행파일이 없다 — 무선 HTTPS 에 필요하다. "
            "설치하거나 certfile/keyfile 을 직접 넘겨라") from exc
    if res.returncode != 0:
        raise RuntimeError(f"인증서 생성 실패: {res.stderr.strip()}")
    key.chmod(0o600)
    stamp.write_text(san)
    return str(cert), str(key)
