#!/bin/sh

if [ $# -ne 2 ] ; then
    echo "Usage: $0 bin_file key_file"
    exit
fi

set -x

call_path=$(pwd)
script_path=$(dirname "$0")
#script_path="${call_path}/../../../"
bin_file=$1
bin_basename=$(basename "${bin_file}" .bin)
key_file=$2
key_path=$(dirname "${key_file}")
key_basename=$(basename "${key_file}" .pem)

#echo call_path: $call_path
#echo script_path: $script_path
#echo bin_file: $bin_file
#echo bin_basename: $bin_basename
#echo key_file: $key_file
#echo key_basename: $key_basename

# To generate ECC P-256 private key
if [ ! -f "${key_file}" ] ; then
    "${script_path}/imgtool" keygen -k "${key_file}" -t ecdsa-p256
fi

# To generate ECC P-256 public key
if [ ! -f "${key_path}/${key_basename}.pub" ] ; then
    "${script_path}/imgtool" getpub -k "${key_file}" > "${key_path}/${key_basename}.pub"
fi

# To sign image
"${script_path}/imgtool" sign -k "${key_file}" --public-key-format hash --align 4 -v 1.2.3 -H 0x800 --pad-header -S 0x40000 -s 0 "${bin_file}" "${script_path}/${bin_basename}_signed.bin"

set +x