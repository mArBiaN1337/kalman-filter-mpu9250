. PHONY: all 

roll: kf_filter.py
	python kf_filter.py 0

pitch: kf_filter.py
	python kf_filter.py 1