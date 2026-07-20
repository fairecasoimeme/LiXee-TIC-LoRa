#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ZLinky OTA sender - pousse un firmware .bin sur un ZLinky via le receiver Arduino.

Architecture:
  PC --(Serial USB)--> Receiver Arduino (gateway) --(LoRa)--> ZLinky

Le receiver doit etre dans un mode "OTA gateway" qui accepte des commandes
serie pour transmettre des paquets LoRa et retourne les ACK.

Protocol cote ZLinky (cf app_lora_ota.h):
  OTA_START   17B (gw->dev): type+seq+ver(2)+size(4)+crc(4)+blocks(2)+block_size(1) + 2B MIC
  OTA_BLOCK  ~70B (gw->dev): type+seq+block_idx(2) + data(64) + 2B MIC
  OTA_END      9B (gw->dev): type+seq+full_crc(4)+reserved(1) + 2B MIC
  OTA_ACK      9B (dev->gw): type+seq+block_idx(2)+status(1)+reserved(2) + 2B MIC

Note: les paquets sont chiffres AES-CTR cote ZLinky avec MIC CMAC tronque.
      L'outil envoie le packet en clair, le receiver Arduino chiffre + transmet.

Usage :
  python ota_send.py firmware.bin --port COM5 --target-mac 00158D0005D2ABE1

  --port           Port serie du receiver Arduino (gateway)
  --target-mac     MAC du ZLinky destinataire (8 bytes hex)
  --new-version   Version du nouveau firmware (uint16, default = auto)
  --block-size    Taille block en bytes (default 64)
  --baud          Baudrate serial (default 115200)
  --dry-run       N'envoie rien, calcule seulement les CRC + blocks
"""

import argparse
import os
import struct
import sys
import time
import zlib

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)


# ====== Protocole constants (must match app_lora_ota.h) ======
LORA_TIC_PROTOCOL_VERSION_OTA = 0x01

LORA_TIC_PKT_OTA_START   = 0x06
LORA_TIC_PKT_OTA_BLOCK   = 0x07
LORA_TIC_PKT_OTA_END     = 0x08
LORA_TIC_PKT_OTA_ACK     = 0x09
LORA_TIC_PKT_OTA_STATUS  = 0x0A

DEFAULT_BLOCK_SIZE = 64

# Status codes
LORA_OTA_STATUS_OK              = 0x00
LORA_OTA_STATUS_BUSY            = 0x01
LORA_OTA_STATUS_BLOCK_BAD_CRC   = 0x02
LORA_OTA_STATUS_FLASH_ERROR     = 0x03
LORA_OTA_STATUS_OUT_OF_RANGE    = 0x04
LORA_OTA_STATUS_FW_TOO_BIG      = 0x05
LORA_OTA_STATUS_FW_BAD_CRC      = 0x06

ACK_TIMEOUT_S       = 5.0
MAX_RETRIES_BLOCK   = 10   # tolerant aux collisions LoRa avec d'autres devices


def crc32_of_file(path):
    """Compute CRC32 of an entire file."""
    crc = 0
    with open(path, "rb") as f:
        while True:
            chunk = f.read(4096)
            if not chunk:
                break
            crc = zlib.crc32(chunk, crc)
    return crc & 0xFFFFFFFF


def split_blocks(path, block_size):
    """Yield (block_idx, data) tuples reading firmware in chunks."""
    idx = 0
    with open(path, "rb") as f:
        while True:
            chunk = f.read(block_size)
            if not chunk:
                break
            # Pad last block with 0xFF if not aligned
            if len(chunk) < block_size:
                chunk = chunk + b"\xff" * (block_size - len(chunk))
            yield (idx, chunk)
            idx += 1


def build_ota_start(seq, new_ver, fw_size, fw_crc, total_blocks, block_size):
    """Pack OTA_START (15 bytes raw, sans MIC)."""
    pkt = bytes([
        (LORA_TIC_PROTOCOL_VERSION_OTA << 4) | LORA_TIC_PKT_OTA_START,
        seq,
    ])
    pkt += struct.pack(">HIIHB", new_ver, fw_size, fw_crc, total_blocks, block_size)
    return pkt  # 2 + 2 + 4 + 4 + 2 + 1 = 15


def build_ota_block(seq, block_idx, data):
    """Pack OTA_BLOCK : 4-byte header + data."""
    pkt = bytes([
        (LORA_TIC_PROTOCOL_VERSION_OTA << 4) | LORA_TIC_PKT_OTA_BLOCK,
        seq,
        (block_idx >> 8) & 0xFF,
        block_idx & 0xFF,
    ])
    pkt += data
    return pkt  # 4 + 64 = 68


def build_ota_end(seq, full_crc):
    """Pack OTA_END (7 bytes raw)."""
    pkt = bytes([
        (LORA_TIC_PROTOCOL_VERSION_OTA << 4) | LORA_TIC_PKT_OTA_END,
        seq,
    ])
    pkt += struct.pack(">IB", full_crc, 0x00)
    return pkt  # 2 + 4 + 1 = 7


def parse_ack(data):
    """Parse OTA_ACK packet (decrypted, MIC stripped)."""
    if len(data) < 7:
        return None
    type_byte = data[0] & 0x0F
    if type_byte != LORA_TIC_PKT_OTA_ACK:
        return None
    seq = data[1]
    block_idx = (data[2] << 8) | data[3]
    status = data[4]
    return {"seq": seq, "block_idx": block_idx, "status": status}


# ====== Gateway interface (Serial) ======

class OtaGateway:
    """Wraps the Arduino receiver acting as OTA TX/RX gateway via Serial."""

    def __init__(self, port, baud, target_mac):
        self.ser = serial.Serial(port, baud, timeout=ACK_TIMEOUT_S)
        time.sleep(2)  # Arduino reset apres open
        self.ser.reset_input_buffer()
        self.target_mac = target_mac

    def __enter__(self):
        return self

    def __exit__(self, *args):
        # Sortie propre du mode OTA_GATEWAY pour que le receiver reprenne
        # son mode RX_PAIRED normal et continue d'ecouter les autres emitters.
        try:
            self.ser.write(b"OTA_GW_EXIT\n")
            # On attend brievement OTA_GW_EXITED, sans bloquer si le receiver
            # est crashe / non reactif. Pas grave si on rate la confirmation.
            deadline = time.time() + 1.0
            while time.time() < deadline:
                line = self.ser.readline().decode(errors="ignore").strip()
                if "OTA_GW_EXITED" in line:
                    print("  Gateway returned to normal RX mode.")
                    break
                if line:
                    print("  [gw]", line)
        except Exception:
            pass
        self.ser.close()

    def send_setup(self):
        """Tell receiver to enter OTA gateway mode + set target MAC."""
        cmd = "OTA_GW_INIT " + self.target_mac.hex().upper() + "\n"
        self.ser.write(cmd.encode())
        # Wait acknowledge (OTA_GW_READY)
        for _ in range(50):
            line = self.ser.readline().decode(errors="ignore").strip()
            if "OTA_GW_READY" in line:
                return True
            if line:
                print("  [gw]", line)
        return False

    def send_packet(self, pkt_bytes):
        """Send a raw OTA packet via Serial. Receiver chiffre + TX LoRa.
        Format Serial : 'OTA_TX <hex>\n'
        Reponse : 'OTA_RX <hex>\n' (ACK recu) ou 'OTA_TIMEOUT\n' """
        cmd = "OTA_TX " + pkt_bytes.hex().upper() + "\n"
        self.ser.write(cmd.encode())

        # Lecture reponse
        deadline = time.time() + ACK_TIMEOUT_S
        while time.time() < deadline:
            line = self.ser.readline().decode(errors="ignore").strip()
            if line.startswith("OTA_RX"):
                hex_ack = line.split(" ", 1)[1] if " " in line else ""
                try:
                    return bytes.fromhex(hex_ack)
                except ValueError:
                    return None
            if "OTA_TIMEOUT" in line:
                return None
            if line:
                # Forward debug from gateway
                print("  [gw]", line)
        return None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("firmware", help="Path to firmware .bin")
    p.add_argument("--port", required=True, help="Serial port of gateway (e.g. COM5 or /dev/ttyUSB0)")
    p.add_argument("--target-mac", required=True, help="ZLinky MAC (16 hex chars)")
    p.add_argument("--new-version", type=lambda x: int(x, 0), default=None,
                   help="New firmware version, decimal or hex 0x... (default: auto)")
    p.add_argument("--block-size", type=int, default=DEFAULT_BLOCK_SIZE)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--dry-run", action="store_true", help="Don't send anything, just compute")
    args = p.parse_args()

    fw_path = args.firmware
    if not os.path.isfile(fw_path):
        print(f"ERROR: file not found: {fw_path}")
        sys.exit(1)

    fw_size = os.path.getsize(fw_path)
    fw_crc = crc32_of_file(fw_path)
    block_size = args.block_size
    total_blocks = (fw_size + block_size - 1) // block_size
    new_ver = args.new_version or 0x0001

    target_mac_bytes = bytes.fromhex(args.target_mac)
    if len(target_mac_bytes) != 8:
        print("ERROR: target-mac doit etre 8 bytes (16 hex chars)")
        sys.exit(1)

    print("=" * 60)
    print("ZLinky OTA sender")
    print("=" * 60)
    print(f"Firmware     : {fw_path}")
    print(f"Size         : {fw_size} bytes")
    print(f"CRC32        : 0x{fw_crc:08X}")
    print(f"Version      : 0x{new_ver:04X}")
    print(f"Block size   : {block_size} bytes")
    print(f"Total blocks : {total_blocks}")
    print(f"Target MAC   : {args.target_mac}")
    print(f"Port         : {args.port} @ {args.baud}")

    # Estimation duree
    avg_block_time_ms = 250  # SF11 + ACK + delay
    total_min = (total_blocks * avg_block_time_ms) / 60000
    print(f"Estimated transfer time : {total_min:.1f} min @ SF11 (250 ms/block)")
    print("=" * 60)

    if args.dry_run:
        print("DRY RUN - nothing sent.")
        return 0

    seq = 0

    with OtaGateway(args.port, args.baud, target_mac_bytes) as gw:
        if not gw.send_setup():
            print("ERROR: gateway setup failed (check port and receiver firmware)")
            return 1

        # 1. OTA_START
        print(f"\n[1/3] Sending OTA_START...")
        pkt = build_ota_start(seq, new_ver, fw_size, fw_crc, total_blocks, block_size)
        ack = gw.send_packet(pkt)
        if not ack:
            print("ERROR: no ACK on OTA_START")
            return 1
        ack_info = parse_ack(ack)
        if not ack_info or ack_info["status"] != LORA_OTA_STATUS_OK:
            print(f"ERROR: OTA_START rejected: {ack_info}")
            return 1
        print(f"  OK : ZLinky ready to receive {total_blocks} blocks")
        seq = (seq + 1) & 0xFF

        # 2. OTA_BLOCK loop — index-based pour pouvoir sauter des blocs deja
        # acceptes (ACK perdu cote air mais ZLinky les a bien recus).
        print(f"\n[2/3] Sending {total_blocks} blocks...")
        t_start = time.time()
        all_blocks = list(split_blocks(fw_path, block_size))
        i = 0
        while i < len(all_blocks):
            block_idx, data = all_blocks[i]
            advanced = False
            for retry in range(MAX_RETRIES_BLOCK):
                pkt = build_ota_block(seq, block_idx, data)
                ack = gw.send_packet(pkt)
                if ack:
                    ack_info = parse_ack(ack)
                    if ack_info:
                        if ack_info["status"] == LORA_OTA_STATUS_OK and ack_info["block_idx"] == block_idx:
                            break
                        # NACK OUT_OF_RANGE avec block_idx > current : le ZLinky
                        # a deja recu ce bloc (et plus). On saute jusqu'a son
                        # prochain bloc attendu.
                        if (ack_info["status"] == LORA_OTA_STATUS_OUT_OF_RANGE
                                and ack_info["block_idx"] > block_idx
                                and ack_info["block_idx"] < total_blocks):
                            print(f"  ZLinky ahead at block {ack_info['block_idx']} "
                                  f"(was sending {block_idx}), skipping")
                            i = ack_info["block_idx"]
                            advanced = True
                            break
                    print(f"  block {block_idx} retry {retry+1}: {ack_info}")
                else:
                    print(f"  block {block_idx} retry {retry+1}: timeout")
            else:
                print(f"ERROR: max retries on block {block_idx}")
                return 1

            seq = (seq + 1) & 0xFF
            if not advanced:
                i += 1
            if (block_idx % 10) == 0:
                pct = 100 * (block_idx + 1) / total_blocks
                elapsed = time.time() - t_start
                rate = (block_idx + 1) / elapsed if elapsed > 0 else 0
                eta = (total_blocks - block_idx - 1) / rate if rate > 0 else 0
                print(f"  block {block_idx + 1}/{total_blocks}  {pct:.1f}%  {rate:.1f} blk/s  ETA {eta:.0f}s")

        elapsed = time.time() - t_start
        print(f"  All blocks sent in {elapsed:.1f}s ({total_blocks/elapsed:.1f} blk/s)")

        # 3. OTA_END
        print(f"\n[3/3] Sending OTA_END...")
        pkt = build_ota_end(seq, fw_crc)
        ack = gw.send_packet(pkt)
        if not ack:
            print("ERROR: no ACK on OTA_END")
            return 1
        ack_info = parse_ack(ack)
        if ack_info and ack_info["status"] == LORA_OTA_STATUS_OK:
            print("  OK : ZLinky committing + rebooting now")
            print("\n*** OTA UPDATE COMPLETE ***")
            return 0
        else:
            print(f"ERROR: OTA_END rejected: {ack_info}")
            return 1


if __name__ == "__main__":
    sys.exit(main())
