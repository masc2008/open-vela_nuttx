#!/usr/bin/env python3
############################################################################
# tools/coredump.py
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

import argparse
import base64
import binascii
import io
import mmap
import os
import struct
import sys

try:
    import lzf
except ImportError:
    print("lzf module not found, execute `pip install python-lzf`")


def try_extract(filepath) -> io.BytesIO:
    """
    Parse log file lines, and try to extract coredump data text
    Return extracted coredump binary data or None if not found

    Note: if multiple core dump found in file, only the first one is extracted
    """
    in_coredump = False
    bas64 = False
    coredump = []
    with open(filepath, "r", errors="ignore") as f:
        for line in f.readlines():
            line = line.rstrip("\n\r")
            if "Start coredump" in line:
                in_coredump = True
                print(f"Found coredump start marker at '{line}'")
                if coredump:
                    print("Warning: multiple core dump found.")
                    break
            elif "Finish coredump" in line:
                bas64 = "base64 formatted" in line
                line = line.strip("\n\r")
                print(f"Found finish coredump marker at '{line}'")
                in_coredump = False
            elif in_coredump:
                index = line.rfind(" ")
                if index > 0:
                    line = line[index + 1 :]
                if line:
                    coredump += line

    print(f"Found {len(coredump)} lines of coredump data")

    try:
        return coredump and io.BytesIO(
            base64.b64decode("".join(coredump))
            if bas64
            else binascii.unhexlify("".join(coredump))
        )
    except Exception:
        return None


def try_decompress(input: io.BytesIO) -> bytearray:
    output = bytearray()
    while True:
        prefix = input.read(2)  # 0, 1
        typ = input.read(1)  # 2
        clen = input.read(2)  # 3, 4
        if prefix != b"ZV":
            break

        clen = struct.unpack(">H", clen)[0]
        if typ == b"\x00":
            chunk = input.read(clen)
            output.extend(chunk)
        elif typ == b"\x01":
            uncompressed_len = struct.unpack(">H", input.read(2))[0]
            cdata = input.read(clen)
            chunk = lzf.decompress(cdata, uncompressed_len)
            output.extend(chunk)
        else:
            break

    input.seek(0)
    return output or input.read()


def mmap_file(file_path, size=None):
    with open(file_path, "rb") as f:
        fd = f.fileno()
        if size is None:
            size = os.fstat(fd).st_size
        return mmap.mmap(fd, size, access=mmap.ACCESS_READ)

def is_text_file(path, sample_size=2048):
    with open(path, 'rb') as f:
        sample = f.read(sample_size)
    try:
        sample.decode('utf-8')
        return True
    except UnicodeDecodeError:
        return False

def parse_args():
    global args
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        allow_abbrev=False,
    )
    parser.add_argument("input")
    parser.add_argument("-o", "--output", help="Output core file.")
    parser.add_argument(
        "-b",
        "--binary",
        action="store_true",
        default=False,
        help="Treat input file as binary data.",
    )
    args = parser.parse_args()

def extract_coredump_section(input_file, output_file=None):
    # 非文本（或显式指定二进制）则直接返回原文件路径
    if args.binary or not is_text_file(input_file):
        return input_file, False  # False 表示后续按二进制处理

    if output_file is None:
        base_name = os.path.splitext(input_file)[0]
        output_file = f"{base_name}.txt"

    start_marker = "coredump_dump_syslog: Start coredump:"
    end_marker = "coredump_dump_syslog: Finish coredump"

    try:
        with open(input_file, 'r', encoding='utf-8') as infile, open(output_file, 'w') as outfile:
            inside_coredump = False
            coredump_lines = []

            for line in infile:
                if start_marker in line:
                    inside_coredump = True
                    continue

                if end_marker in line:
                    inside_coredump = False
                    break

                if inside_coredump:
                    stripped_line = line.strip()
                    if stripped_line:
                        coredump_lines.append(line)

            if coredump_lines:
                outfile.writelines(coredump_lines)
                print(f"Coredump section successfully extracted to {output_file}")
                return output_file, True  # True 表示文本路径
            else:
                print("No coredump section found in the input file", file=sys.stderr)
                return input_file, False
    except IOError as e:
        print(f"Error accessing files: {e}", file=sys.stderr)
        return None, False


def main():
    parse_args()

    if not os.path.exists(args.input):
        print(f"Error: Input file {args.input} does not exist.")
        sys.exit(1)

    output = args.output or f"{args.input}.core"

    extracted = try_extract(args.input)
    if extracted is None:
        print(f"Error: none hex or base64 content in {args.input}.")
        sys.exit(1)

    input = mmap_file(args.input) if args.binary or not extracted else extracted
    coredump = try_decompress(input)
    with open(output, "wb") as outfile:
        outfile.write(coredump)

    print("Core file conversion completed: " + output)


if __name__ == "__main__":
    main()
