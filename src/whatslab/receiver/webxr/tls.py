from __future__ import annotations

import datetime
import ipaddress
import os
import socket
from pathlib import Path
from typing import List, Tuple

from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.x509.oid import NameOID

CACHE_DIR = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "whatslab" / "webxr"
CERT_DAYS = 3650


def preferred_ip() -> str:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        if sock.connect_ex(("1.1.1.1", 53)) != 0:
            return "127.0.0.1"
        return sock.getsockname()[0]


def _san(ips: List[str]) -> x509.SubjectAlternativeName:
    hosts = ["127.0.0.1", "::1", *ips]
    return x509.SubjectAlternativeName(
        [x509.DNSName("localhost"), *(x509.IPAddress(ipaddress.ip_address(h)) for h in hosts)]
    )


def ensure_cert(ips: List[str] = None, cache_dir: Path = None) -> Tuple[str, str]:
    if ips is None:
        ip = preferred_ip()
        ips = [] if ip.startswith("127.") else [ip]
    cache = CACHE_DIR if cache_dir is None else Path(cache_dir)
    cache.mkdir(parents=True, exist_ok=True)
    cert = cache / "cert.pem"
    key = cache / "key.pem"
    stamp = cache / "san.txt"
    san = ",".join(["localhost", "127.0.0.1", "::1", *ips])

    if cert.exists() and key.exists() and stamp.exists() and stamp.read_text() == san:
        return str(cert), str(key)

    private = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "whatslab-webxr")])
    now = datetime.datetime.now(datetime.timezone.utc)
    certificate = (
        x509.CertificateBuilder()
        .subject_name(name)
        .issuer_name(name)
        .public_key(private.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now - datetime.timedelta(minutes=5))
        .not_valid_after(now + datetime.timedelta(days=CERT_DAYS))
        .add_extension(_san(ips), critical=False)
        .sign(private, hashes.SHA256())
    )
    key.write_bytes(private.private_bytes(
        serialization.Encoding.PEM,
        serialization.PrivateFormat.TraditionalOpenSSL,
        serialization.NoEncryption(),
    ))
    key.chmod(0o600)
    cert.write_bytes(certificate.public_bytes(serialization.Encoding.PEM))
    stamp.write_text(san)
    return str(cert), str(key)
