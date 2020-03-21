#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 16 14:57:07 2019

@author: hiro
"""
import os

def getLatestMatchingFile(path, basename):
    max_mtime = 0
    max_file = ''
    for dirname,subdirs,files in os.walk(path):
        for fname in files:
            full_path = os.path.join(dirname, fname)
            mtime = os.stat(full_path).st_mtime
            if mtime > max_mtime and fname.find(basename) > 0:
                max_mtime = mtime
                max_dir = dirname
                max_file = full_path
    return max_file