import argparse


def get_args(default_settings):

    parser = argparse.ArgumentParser(
        description='A two-wheeled pendulum robot controller')
    parser.add_argument(
        '--bluetooth',
        help="Use bluetooth port, requires MAC address and port number",
        default=[default_settings["bluetooth"]["address"],
                 default_settings["bluetooth"]["port"]],
        metavar=("MAC", "port"),
        nargs=2,
        type=str
    )

    parser.add_argument(
        '--tcp',
        help="Add host tcp port, requires ip address and port number",
        default=[default_settings["tcp"]["address"], default_settings["tcp"]
                 ["port_command"], default_settings["tcp"]["port_log"]],
        metavar=("ip", "cmd-port", "log-port"),
        nargs=3,
        type=str
    )

    parser.add_argument(
        '--simulate',
        help="Run in no-robot mode",
        default=False,
        action='store_true'
    )

    args = parser.parse_args()

    if args.simulate:
        args.bluetooth = [None, None]
        args.tcp = ["127.0.0.1", "11337", "11338"]

    return args


if __name__ == "__main__":
    print("Args:")
    print(get_args())
