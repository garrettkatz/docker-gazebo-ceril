#!/bin/bash

echo "Checking if CERIL is working"

python3 /src/copct/toy_example.py

echo "Testing if gazebo works"

apt-get update && apt-get install -y curl

curl -o double_pendulum.sdf http://models.gazebosim.org/double_pendulum_with_base/model-1_4.sdf

gz model --model-name double_pendulum --spawn-file double_pendulum.sdf

gz log --record 1

sleep 5

gz log --record 0

cd ~/.gazebo/log/*/gzserver/

gz log --step --hz 10 --filter *.pose/*.pose --file state.log
