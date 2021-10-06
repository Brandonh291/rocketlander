#!/bin/bash

cp ./uiuc.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable uiuc.service
systemctl start uiuc.service
