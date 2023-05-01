#!/usr/bin/env python3

'''This script watches a directory for changes, and runs goby_log_tool to convert any new .goby files to .h5 files'''

import argparse
import logging
import os
from posixpath import islink
import time
import glob
import jaialogs

parser = argparse.ArgumentParser()
parser.add_argument('-d', dest="path", type=str, default="/var/log/jaiabot/bot_offload", help="Path to monitor for new goby files to convert")
parser.add_argument("-l", dest='logLevel', type=str, default='INFO', help="Logging level (CRITICAL, ERROR, WARNING, INFO, DEBUG)")
parser.add_argument('-s', dest='size_limit', type=int, default='100000000000', help='Maximum goby file size to convert')
args = parser.parse_args()

# Setup log level
logLevel = getattr(logging, args.logLevel.upper())
logging.basicConfig(level=logLevel)

# Monitor for changes
path = os.path.expanduser(args.path)
logging.info(f'Monitoring path: {path}')


def file_is_newer(filename: str, mtime: float):
    try:
        file_mtime = os.path.getmtime(filename)
    except:
        return False

    return file_mtime > mtime


while True:
    # Get all goby files
    goby_files = glob.glob(f'{path}/**/*.goby', recursive=True)
    goby_file_bodies = [goby_file[:-5] for goby_file in goby_files]

    # Go through each goby file, and see if the h5 file is older or nonexistent
    for file_body in goby_file_bodies:
        goby_filename = file_body + '.goby'

        # Skip symlinks
        if os.path.islink(goby_filename):
            continue

        # Skip giant files
        file_size = os.path.getsize(goby_filename)
        if file_size > args.size_limit:
            continue

        goby_mtime = os.path.getmtime(goby_filename)

        h5_filename = file_body + '.h5'

        if not file_is_newer(h5_filename, goby_mtime):
            try:
                # Generate h5 file
                cmd = f'nice -n 10 goby_log_tool --input_file {goby_filename} --output_file {h5_filename} --format HDF5'
                logging.info(cmd)
                os.system(cmd)

                os.system(f'chgrp jaiaplot {h5_filename}')
                os.system(f'chmod ug+rw {h5_filename}')

            except FileNotFoundError:
                logging.warning(f'File not found: {goby_filename}')
                continue

        kmz_filename = file_body + '.kmz'

        if not file_is_newer(kmz_filename, goby_mtime):
            logging.info(f'Generating {kmz_filename}')
            jaialogs.generate_kmz(h5_filename, kmz_filename)


    time.sleep(30)