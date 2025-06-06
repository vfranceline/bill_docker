#!/bin/bash

sudo apt update

sudo xargs apt install -y < requirements_apt.txt

