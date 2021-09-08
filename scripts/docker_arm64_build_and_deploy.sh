#!/bin/bash

script_dir=$(dirname $0)

if [ -z "$1" ]
    then
        echo "Enter a host to deploy to"
        exit
fi

cd ${script_dir}/..

LAST_BUILD_DATE=$(date -r arm64_build +%s)
ONE_WEEK_LATER=$(expr "$LAST_BUILD_DATE" + 604800)

NOW=$(date +%s)

if [ $NOW -ge $ONE_WEEK_LATER ];
    then
        echo "****It's been a while since we updated packages in the container, so let's take care of that now."
        sudo docker run -v `pwd`:/home/ubuntu/jaiabot -w /home/ubuntu/jaiabot -t gobysoft/jaiabot-ubuntu-arm64:20.04.1 bash -c "apt update && apt upgrade -y && ./scripts/arm64_build.sh"
    else
        echo "****Up to date - let's get straight to compiling, shall we?"
        sudo docker run -v `pwd`:/home/ubuntu/jaiabot -w /home/ubuntu/jaiabot -t gobysoft/jaiabot-ubuntu-arm64:20.04.1 bash -c "./scripts/arm64_build.sh"
fi  

echo "rsync build/bin and build/lib"
rsync -aP arm64_build/bin arm64_build/lib ubuntu@$1:/home/ubuntu/jaiabot/build
echo "rsync ../jaiabot-configuration"
rsync -aP ../jaiabot-configuration ubuntu@$1:/home/ubuntu/
