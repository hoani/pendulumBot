

init:
	(\
		python3 -m venv ./env; \
		source "./env/bin/activate"; \
		pip3 install -r "./requirements.txt" \
	)

test:
	(\
		source "./env/bin/activate"; \
		python3 -m pytest; \
	)

.PHONY: init test