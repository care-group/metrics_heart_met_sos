#!/bin/bash

RASA_DIR=$HOME/catkin_ws/src/metrics/metrics_rasa/src/rasa
cd $RASA_DIR
rasa run --enable-api
