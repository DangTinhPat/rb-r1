#!/usr/bin/env python3
"""Upload an ESP32-S3 application to the bench over Wi-Fi (HTTP POST /ota on port 8081).

Works only while the bench firmware (rb-r1-check-motor) is running. The first upload from another firmware goes over USB
or Arduino OTA: see docs/OTA.md.
Credentials stay in .local/ota.json and are never passed as process arguments.
The firmware must confirm that all six motors are OFF before flashing.
"""
import argparse
import hashlib
import json
import struct
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def image_identity(data):
    """Reject anything that is not an ESP32-S3 app image; return its hashes."""
    if (len(data) < 288 or len(data) > 0x640000 or data[0] != 0xe9 or
            struct.unpack_from('<H', data, 12)[0] != 9 or
            struct.unpack_from('<I', data, 32)[0] != 0xabcd5432):
        raise ValueError(
            'Expected ESP32-S3 app binary (not bootloader, partition table or merged flash image)')
    return {'elf_sha256': data[176:208].hex(),
            'project': data[80:112].split(b'\0')[0].decode(),
            'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data)}


def http(host, key, path, data=None, timeout=4):
    request = urllib.request.Request(
        f'http://{host}:8081{path}', data=data,
        headers={'X-OTA-Key': key, 'Content-Type': 'application/octet-stream'})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return response.read()


def status(host, credentials):
    """State of the running bench firmware, from its HTTP status endpoint."""
    try:
        state = json.loads(http(host, credentials['password'], '/status', timeout=2))
    except (OSError, ValueError) as exc:
        raise RuntimeError(
            f'No answer from the bench firmware at {host}:8081 ({exc}). Is rb-r1-check-motor running? '
            'The first upload from another firmware must go over USB or Arduino OTA (docs/OTA.md).') from exc
    if state.get('firmware') != 'rb-r1-check-motor':
        raise RuntimeError(f"Unexpected firmware '{state.get('firmware')}'")
    if state['mac'].lower() != credentials['mac'].lower():
        raise RuntimeError('MCU MAC identity mismatch')
    return state


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--host')
    ap.add_argument('--binary', type=Path,
                    default=ROOT / 'firmware/.pio/build/check-motor-ota/firmware.bin')
    ap.add_argument('--credentials', type=Path, default=ROOT / '.local/ota.json')
    ap.add_argument('--status', action='store_true')
    ap.add_argument('--check', action='store_true', help='Validate local image only')
    args = ap.parse_args()
    credentials = json.loads(args.credentials.read_text())
    host = args.host or credentials['host']
    if args.check:
        print(json.dumps(image_identity(args.binary.read_bytes()), indent=2))
        return

    before = status(host, credentials)
    if args.status:
        print(json.dumps(before, indent=2))
        return

    data = args.binary.read_bytes()
    identity = image_identity(data)
    print('Image:', json.dumps(identity), flush=True)
    if any(m['enabled'] for m in before['motors']):
        raise RuntimeError('STOP motors in GUI before upload')
    print(http(host, credentials['password'], '/ota', data, timeout=60).decode().strip(), flush=True)

    end = time.monotonic() + 50
    while time.monotonic() < end:
        try:
            after = status(host, credentials)
            if (after['elf_sha256'] == identity['elf_sha256'] and
                    after['partition'] != before['partition']):
                if after['maintenance'] or any(m['enabled'] for m in after['motors']):
                    raise RuntimeError('Unexpected motor/maintenance state after boot')
                print('PASS: same MCU, expected image, switched OTA slot, outputs disabled', flush=True)
                print(json.dumps(after, indent=2))
                return
        except (OSError, RuntimeError, ValueError):
            pass
        time.sleep(.5)
    raise RuntimeError('Transfer finished; expected reboot/image not confirmed')


if __name__ == '__main__':
    main()
