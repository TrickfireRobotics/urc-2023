#!/usr/bin/env bash

# SSH into the Orin device without password prompt, using sshpass
# if the SSH_PASS environment variable is set. The SSH connection is configured to disable
#  GSSAPI authentication, suppress log messages, and ignore host key checking for convenience.

ORIN_IP="192.168.0.146"

TERM=xterm-256color ssh -Y -t \
    -o GSSAPIAuthentication=no -o LogLevel=ERROR -o StrictHostKeyChecking=no \
    trickfire@"$ORIN_IP" "clear; cd ~/urc-2023 2>/dev/null; bash"
