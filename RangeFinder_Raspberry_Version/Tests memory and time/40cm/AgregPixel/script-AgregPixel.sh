#!/bin/bash

cont=1
while [ $cont -lt 11 ];
do
    echo $cont
    endTxt="Profile-AgregPixel-"$cont".txt"
	echo $endTxt
	python -m memory_profiler "AgregPixel"".py" "$cont"".jpg" ls >> $endTxt
	kernprof -l -v "AgregPixel"".py" "$cont"".jpg" ls >> $endTxt
    let cont=cont+1
done