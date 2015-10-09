#!/bin/sh

sudo /etc/init.d/omniorb4-nameserver stop
sleep 1
sudo rm /var/lib/omniorb/*
sleep 1
sudo /etc/init.d/omniorb4-nameserver start
