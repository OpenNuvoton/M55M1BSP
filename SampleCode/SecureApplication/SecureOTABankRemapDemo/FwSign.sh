#!/bin/sh
# https://docs.mcuboot.com/imgtool.html

if [ $# -lt 2 ] ; then
    echo "Usage: $0 bin_file key_file [fw_ver]"
    exit
fi

set -x	# Enable debug mode (Print each command before executing it.)

call_path=$(pwd)
script_path=$(dirname "$0")
#script_path="${call_path}/../../../"
bin_file=$1
bin_basename=$(basename "${bin_file}" .bin)
key_file=$2
key_path=$(dirname "${key_file}")
key_basename=$(basename "${key_file}" .pem)

if [ $# -eq 2 ]; then
    fw_ver=0
else
    fw_ver=$3
fi

#echo call_path: $call_path
#echo script_path: $script_path
#echo bin_file: $bin_file
#echo bin_basename: $bin_basename
#echo key_file: $key_file
#echo key_basename: $key_basename

# To generate ECC P-256 private key
if [ ! -f "${key_file}" ] ; then
    "${script_path}/../../../Tool/imgtool" keygen -k "${key_file}" -t ecdsa-p256
fi

# To generate ECC P-256 public key
if [ ! -f "${key_path}/${key_basename}_pub.c" ] ; then
    "${script_path}/../../../Tool/imgtool" getpub -k "${key_file}" > "${key_path}/${key_basename}_pub.c"
fi

# To sign image
"${script_path}/../../../Tool/imgtool" sign -k "${key_file}" --public-key-format hash --align 4 -v 1.2.3 -H 0x400 --pad-header -S 0x40000 -s ${fw_ver} "${bin_file}" "${script_path}/${bin_basename}_signed.bin"

# Align signed image file size to 4 bytes for OTA App
"${script_path}/../srec_cat.exe" "${script_path}/${bin_basename}_signed.bin" -binary \
-fill 0x00 -within "${script_path}/${bin_basename}_signed.bin" -binary -range-padding 4 \
-o "${script_path}/${bin_basename}_signed.bin" -binary

set +x	# Disable debug mode
