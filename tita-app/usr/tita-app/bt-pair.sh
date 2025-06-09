#!/bin/bash

sudo hciconfig hci0 name $(cat config.json | grep 'mqtt_username' | awk -F'"' '{print $4}')

{
    echo "menu gatt"
    echo "register-service aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee"
    sleep 1
    echo "yes"
    echo "register-characteristic bbbbbbbb-cccc-dddd-eeee-ffffffffffff write"
    sleep 1
    echo "00"
    echo "register-application"
    sleep 1
    echo "list-attributes local"
    sleep 1
    echo "back"
    sleep 1
    echo "menu advertise"
    sleep 1
    echo "name $(cat config.json | grep 'mqtt_username' | awk -F'"' '{print $4}')"
    sleep 1
    echo "back"
    sleep 1
    echo "advertise off"
    sleep 1
    echo "advertise on"
    sleep 1
    echo "discoverable on"
    sleep 1
    echo "show"
    sleep 30
    echo "yes"
    sleep 30
    echo "yes"
    while true; do sleep 60; done
} | sudo bluetoothctl
