

init:
	(\
		python3 -m venv ./env; \
		source "./env/bin/activate"; \
		pip3 install --upgrade pip \
		pip3 install -r "./requirements.txt" \
	)

test:
	(\
		python3 -m pytest; \
	)

run: 
	(\
		python3 manualGui.py; \
	)

.PHONY: init test