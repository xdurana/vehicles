#!/bin/sh

cd src
for i in `seq 6665 6676` ; do
    (./epuck $i &)
done