#!/usr/bin/env python3
#
# Copyright © 2019 Hoani Bryson
# License: MIT (https://mit-license.org/)
#
# Command Register
#
# Delegates specific commands to callbacks
#

class CommandRegister:
    def __init__(self):
        self.registry = dict()

    def add(self, command, callback):
        self.registry[command] = callback

    def execute(self, command, payload):
        if command in self.registry.keys():
            return self.registry[command](payload)
        else:
            return False


if __name__ == "__main__":
    print("module not callable")
