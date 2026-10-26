#!/usr/bin/env bash
# Installs every services/*.conf into supervisor and reloads it.
# Run with sudo from any directory.
set -e
cd "$(dirname "$0")/../services"
chmod +x *.conf
cp *.conf /etc/supervisor/conf.d/
supervisorctl reread
supervisorctl update
