#!/bin/bash

# Put the streamtest program in a named tmux session for the pi user
# assume we're invoked by root
#
# Make this appropriate for rc.local in noobs
#
sudo -u pi tmux new-session -d -s chickcam /home/pi/chickcam/streamtest.py
