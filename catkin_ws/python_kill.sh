#!/bin/bash 

ps -ef | grep computer.py | grep python | awk '{print $2}' | xargs -i kill -9 {}

ps -ef | grep player.py | grep python | awk '{print $2}' | xargs -i kill -9 {}
