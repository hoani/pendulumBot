from app import runner
import json


def get_settings(settings_file_path='app/config/settings.json'):
    with open(settings_file_path, "r") as settings_file:
        settings = json.load(settings_file)
    return settings


settings = get_settings()
_runner = runner.RobotRunner(settings)
delta_ms = 20
_runner.run(delta_ms)
print("\nBye BeagleBone!")
