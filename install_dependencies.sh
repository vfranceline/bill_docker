#!/bin/bash

sudo apt update

sudo xargs apt install -y < dependencies/requirements_apt.txt

