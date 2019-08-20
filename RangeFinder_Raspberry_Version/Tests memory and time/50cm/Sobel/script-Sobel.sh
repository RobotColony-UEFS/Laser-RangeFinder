#!/bin/bash

cont=1
while [ $cont -lt 11 ];
do
    echo $cont
    endTxt="Profile-Sobel-"$cont".txt"
	echo $endTxt
	python -m memory_profiler "Sobel"".py" "$cont"".jpg" ls >> $endTxt
	kernprof -l -v "Sobel"".py" "$cont"".jpg" ls >> $endTxt
    let cont=cont+1
done