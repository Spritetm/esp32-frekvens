#!/bin/bash

#Box around the eye; the bit that you want to have show up on the LED screen
EYE_COORDS="289x289+140+98"

rm -f cropped.png eye.raw cropped.raw
for x in *.tif; do
	#Cut out bit around eye.
	convert $x -crop "$EYE_COORDS" cropped.png
	#Tweak brightness/contrast, resize to 16x16 and convert to a raw grayscale file
	convert cropped.png -brightness-contrast -15x30% -resize 16x16 -channel B -separate -depth 8 gray:cropped.raw
	#Add to general file store
	cat cropped.raw >> eye.raw
	#Check where the first red dot is in the det/ image
	l=`convert det/${x%.tif}.png txt: | grep '#FF0000FF' | head -n 1 | sed 's/[,:]/ /g'`
	read x y c <<< $l
	#Top left / right are blink; the assumption is that the pupil coordinates are the same as the
	#non-blink image before.
	if [ $y -lt 30 ]; then
		if [ $x -lt 200 ]; then
			c=1
		else
			c=2
		fi
		x=$px
		y=$py
	else
		c=0
		px=$x
		py=$y
	fi
	echo -e "\t{$x, $y, $c},"
done

