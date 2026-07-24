#!/usr/bin/env python3
import sys
import os
import subprocess
import time

# ==============================================================================
# ?? CRA Compliance Notice:
# 1. DO NOT use sample or automatically generated keys in a production environment.
#    Production keys must be stored securely (e.g., HSM/KMS) and never committed
#    to source control.
# 2. CRA mandates downgrade protection. The Security Counter (-s) MUST be
#    incremented appropriately for every released security update.
# ==============================================================================
#
# Usage: FwSign.py bin_file key_file [fw_ver]

def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <bin_file> <key_file> [security_counter]")
        print(f"  e.g., {sys.argv[0]} app.bin dev_key.pem 1")
        sys.exit(1)

    bin_file = sys.argv[1]
    key_file = sys.argv[2]

    # Retry once because the output binary may not be ready immediately.
    if not os.path.isfile(bin_file):
        print(f"[WARNING] Input image '{bin_file}' not found. Retry after 3 seconds...")
        time.sleep(3)
        if not os.path.isfile(bin_file):
            print(f"[ERROR] Input image '{bin_file}' not found after retry.")
            return 1

    # Safe protection logic to ensure anti-rollback capabilities are not bypassed
    if len(sys.argv) > 3:
        fw_ver = sys.argv[3]
    else:
        print("[CRA WARNING] No security counter provided. Defaulting to '0'. Ensure this value increases on production release.")
        fw_ver = "0"

    call_path = os.getcwd()
    script_path = os.path.dirname(os.path.abspath(__file__))
    bin_basename = os.path.splitext(os.path.basename(bin_file))[0]
    key_path = os.path.dirname(key_file)
    key_basename = os.path.splitext(os.path.basename(key_file))[0]

    imgtool_path = os.path.join(script_path, '../../../Tool/mcuboot/imgtool.py')

    # Generate ECC P-256 private key if not exist
    if not os.path.isfile(key_file):
        print(f"[CRA WARNING] Private key '{key_file}' not found. Generating a DEVELOPMENT-ONLY key.")
        print("   NEVER deploy a production device using this auto-generated file.")
        subprocess.check_call([
            sys.executable, imgtool_path, 'keygen', '-k', key_file, '-t', 'ecdsa-p256'
        ])

    # Generate ECC P-256 public key if not exist
    pubkey_c = os.path.join(key_path, f"{key_basename}_pub.c")
    if not os.path.isfile(pubkey_c):
        print(f"[INFO] Generating public key: {pubkey_c}")
        with open(pubkey_c, 'w') as f:
            subprocess.check_call([
                sys.executable, imgtool_path, 'getpub', '-k', key_file
            ], stdout=f)

    # Sign image
    signed_bin = os.path.join(script_path, f"{bin_basename}_sc{fw_ver}_signed.bin")
    print(f"[INFO] Signing image: {bin_file}\n  -> {signed_bin}")
    subprocess.check_call([
        sys.executable, imgtool_path, 'sign',
        '-k', key_file,
        '--public-key-format', 'hash',
        '--align', '4',
        # -v major.minor.revision+build_num
        # M3351 maps build_num to ROTPK: default/no build_num -> active ROTPK (default ROTPK 0).
        # 11 switches ROTPK 0 -> 1; 12 switches ROTPK 1 -> 2. Switch ROTPK 0 -> 2 is not allowed.
        # Target ROTPK must exist in SOTP (max 3 keys), and rollback is not allowed.
        '-v', '1.0.0+0',
        '-H', '0x400',
        '--pad-header',
        '-S', '0x40000',
        '-s', str(fw_ver),
        bin_file,
        signed_bin
    ])
    signed_bin_size = os.path.getsize(signed_bin)
    padding_size = (-signed_bin_size) % 4
    if padding_size:
        print(f"[INFO] Padding signed image by {padding_size} byte(s) for 4-byte alignment.")
        with open(signed_bin, 'ab') as f:
            f.write(b"\x00" * padding_size)
    print(f"Image successfully signed for development. Output saved to: {signed_bin}")

if __name__ == "__main__":
    main()
