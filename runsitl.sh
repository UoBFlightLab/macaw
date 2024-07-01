#!/bin/bash
ARDUCOPTER=~/mav_ws/arducopter
DEFAULTS=~/mav_ws/copter.parm

$ARDUCOPTER -w --model=quad --home=52.816730,-4.126272,0,180 --defaults $DEFAULTS