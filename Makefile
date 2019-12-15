

init:
	pip3 install pytest numpy

test:
	python3 -m pytest

.PHONY: init test