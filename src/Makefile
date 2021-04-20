# define the name of the virtual environment directory
VENV := venv

# default target, when make executed without arguments
all: venv

$(VENV)/bin/activate: requirements.txt
	pip3 install virtualenv
	python3 -m virtualenv $(VENV)
	./$(VENV)/bin/pip3 install -r requirements.txt

# venv is a shortcut target
venv: $(VENV)/bin/activate

run: venv
	./$(VENV)/bin/python3 plotscript.py $(ARGS)

clean:
	sudo rm -rf $(VENV)/
	find . -type f -name '*.pyc' -delete

.PHONY: all venv run clean
