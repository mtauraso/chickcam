#!/bin/bash

# Put the streamtest program in a named tmux session for the pi user
# assume we're invoked by root this should be runnable from rc.local 
# in noobs
sudo -u pi \
	tmux new-session -d -s chickcam \
	sudo nice -n -10 \
	sudo -u pi \
	"/home/pi/chickcam/streamtest.py" ;
