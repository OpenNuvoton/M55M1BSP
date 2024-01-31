import re
import sys

def find_symbol_address(symbol, text):
    # Use a regular expression to search for the symbol and its address
    # The symbol is now a variable within the regex pattern
    #match = re.search(rf\'{symbol}\s+(0x10\\'[0-9a-f]+)\', text)
    match = re.search(rf'{symbol}\s+(0x[0]*10\'*[0-9a-f]+)', text)
    
    if match:
        # Extract the address and remove the single quote
        address = match.group(1).replace("'", "") 
        return address
    else:
        return None

# Check if a command-line argument has been provided
if len(sys.argv) < 3:
    print("Usage: python script.py ")
    sys.exit(1)  # Exit the script if no argument is provided

# The first command-line argument is the script name, so the second one (index 1) is the file path
input_file_path = sys.argv[1]
output_file_path = sys.argv[2]

# Open the file and read its contents
with open(input_file_path, 'r') as file:
    input_file = file.read()

output_text = f"""
/**************************************************************************//**
 * @file    xomlib.c
 * @version V3.00
 * @brief   Function pointer for XOM APIs.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include "NuMicro.h"

int32_t (*XOM_Add)(int32_t a, int32_t b) = (int32_t (*)(int32_t a, int32_t b))({find_symbol_address("XOM_Add", input_file)});
int32_t (*XOM_Div)(int32_t a, int32_t b) = (int32_t (*)(int32_t a, int32_t b))({find_symbol_address("XOM_Div", input_file)});
int32_t (*XOM_Mul)(int32_t a, int32_t b) = (int32_t (*)(int32_t a, int32_t b))({find_symbol_address("XOM_Mul", input_file)});
int32_t (*XOM_Sub)(int32_t a, int32_t b) = (int32_t (*)(int32_t a, int32_t b))({find_symbol_address("XOM_Sub", input_file)});
int32_t (*XOM_Sum)(int32_t *pbuf, int32_t n) = (int32_t (*)(int32_t *pbuf, int32_t n))({find_symbol_address("XOM_Sum", input_file)});
"""

# Open the output file in write mode
with open(output_file_path, 'w') as output_file:
    output_file.write(output_text)  # Write to file