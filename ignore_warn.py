#!/usr/bin/python3
import sys

blacklist = ['TF_REPEATED_DATA', 'src/buffer_core.cpp']

for line in sys.stdin:
	valid = True
	for phrase in blacklist:
		if(line.find(phrase) != -1):
			valid = False
	if(valid):
		print(line, end='')
