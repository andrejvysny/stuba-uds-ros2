#!/bin/bash

VALUE=$1

ros2 topic pub /mode std_msgs/msg/Int32 "data: $VALUE" --once