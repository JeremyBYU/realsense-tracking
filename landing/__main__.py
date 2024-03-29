import sys
import argparse
import yaml
from pathlib import Path


THIS_FILE = Path(__file__)
THIS_DIR = THIS_FILE.parent
CONFIG_DIR = THIS_DIR.parent / "config" / "landing"
LANDING_CONFIG_FILE = CONFIG_DIR / "landing.yml"


def start(config_file):
    print(config_file)
    # Load yaml file
    with open(config_file) as file:
        config = yaml.safe_load(file)

    # protobuf files are generated dynamically in an architecture specific build directory
    build_dir = THIS_DIR.parent / f"dk-{config['arch']}-build"
    sys.path.insert(1, str(build_dir))

    from landing.LandingService import LandingService
    ls = LandingService(config)
    ls.run()


def parse_args():
    parser = argparse.ArgumentParser(description="Landing Server")
    parser.add_argument('--config', type=str, default=LANDING_CONFIG_FILE)
    args = parser.parse_args()
    return args


def main():
    args = parse_args()
    start(args.config)


if __name__ == "__main__":
    main()
