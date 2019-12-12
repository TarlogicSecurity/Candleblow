from __future__ import print_function
from __future__ import division
from __future__ import unicode_literals
from __future__ import absolute_import
from future import standard_library
standard_library.install_aliases()
from builtins import chr
from builtins import str
from builtins import *
from past.utils import old_div
import os
import argparse
import binascii

from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives import cmac
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

# Output size: 50 kB
DECRYPT_PKT_SIZE = 16
NULL = '' + chr(0x00)
PAD_INIT = '' + chr(0x80)
FOLDER_SEC = 'PL360_ENCFILE/'

##################### LOAD PARAMETERS  ##################################
parser = argparse.ArgumentParser()
parser.add_argument("-in", "--bin_file", help="File name of Binary file to encrypt", required=True)
parser.add_argument("-out", "--out_file", help="Output file name", required=True)
parser.add_argument("-cbc-iv", "--cbc_iv", help="Initialization Vector", required=True)
parser.add_argument("-cbc-key", "--cbc_key", help="Cypher key", required=True)
parser.add_argument("-cmac-key", "--cmac_key", help="Signature key", required=True)
parser.add_argument("-force_ivnbinc", "--force_ivnbinc", help="If it is set, initialization vector and number of blocks must be used in the calculation of the signature", action="store_true")
args = parser.parse_args()

# Create path to store results
if not os.path.exists(FOLDER_SEC):
    os.makedirs(FOLDER_SEC)

# Load binary file
input_bin_file = open(args.bin_file, "rb")
output_bin_sec_file = open(FOLDER_SEC + args.out_file, "wb")
output_bin_sec_file_size = 0
#output_bin_sec_file2 = open(FOLDER_SEC + 'IVNB_' + args.out_file, "wb")

# Load sec configuration
aes_cbc_iv = binascii.unhexlify(args.cbc_iv)
aes_cbc_key = binascii.unhexlify(args.cbc_key)
aes_cmac_key = binascii.unhexlify(args.cmac_key)

# Add padding to source file
data = input_bin_file.read().decode('latin-1')
data_size = len(data)
data_padding = data_size % DECRYPT_PKT_SIZE
data = data + PAD_INIT + NULL * (DECRYPT_PKT_SIZE - data_padding - 1)
data_size = len(data)
num_dec_pkts = old_div(data_size, DECRYPT_PKT_SIZE)

# Encrypt data
backend = default_backend()
cipher = Cipher(algorithms.AES(aes_cbc_key), modes.CBC(aes_cbc_iv), backend)
encryptor = cipher.encryptor()
decryptor = cipher.decryptor()

cipher_text = encryptor.update(data.encode('latin-1'))
cipher_size = len(cipher_text)

cipher_dec_text = decryptor.update(cipher_text)

if(data.encode('latin-1') == cipher_dec_text):
    print('CIPHER OK')
else:
    print('CIPHER ERROR, src_size: ' + str(len(data)) + ' decrypt_size :' + str(len(cipher_dec_text)))

# Get signature
signer = cmac.CMAC(algorithms.AES(aes_cmac_key), backend)
if args.force_ivnbinc:
	cipher_text_for_signature = cipher_text.decode('latin') + aes_cbc_iv.decode('latin') + (NULL * (DECRYPT_PKT_SIZE - 2)) + binascii.unhexlify('{:04x}'.format(num_dec_pkts - 1)).decode('latin')
else:
	cipher_text_for_signature = cipher_text.decode('latin-1')
signer.update(bytes(cipher_text_for_signature, 'latin-1'))
signature = signer.finalize()
sf = open(FOLDER_SEC + 'signature.txt', "w")
line = 'Signature: ' + str(binascii.hexlify(signature))
sf.write(line)
sf.close()
print('See signature in signature.txt file')

# Build ENC file
line = binascii.unhexlify('{:04x}'.format(num_dec_pkts))
output_bin_sec_file.write(line)
line = bytes((NULL * (DECRYPT_PKT_SIZE - 2)), 'latin-1')
output_bin_sec_file.write(line)
line = aes_cbc_iv
output_bin_sec_file.write(line)
line = signature
output_bin_sec_file.write(line)
output_bin_sec_file.write(cipher_text)
output_bin_sec_file.close()  
