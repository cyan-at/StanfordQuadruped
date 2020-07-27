#!/bin/bash

sudo ln -s $(realpath .)/controller.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable controller
sudo systemctl start controller
