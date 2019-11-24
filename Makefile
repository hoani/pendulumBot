

init:
	pip3 install pytest

test:
	python3 -m pytest

.PHONY: init test