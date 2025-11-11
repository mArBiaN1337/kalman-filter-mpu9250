.PHONY: all 

all: roll 

roll: kf_filter.py
	python kf_filter.py roll

pitch: kf_filter.py
	python kf_filter.py pitch

test: kf_filter.py
	python kf_filter.py test

a-roll: box_animation.py
	python box_animation.py roll

a-pitch: box_animation.py
	python box_animation.py pitch

a-test: box_animation.py
	python box_animation.py test

clean:
	rm -f *.pyc
	rm -f __pycache__/*
