import logging
import yaml
from .visualize import Server

logging.basicConfig(level=logging.INFO)

def main():

    try:
        with open("./config/rsserver_default.yaml", 'r') as f:
            config = yaml.safe_load(f)
    except yaml.YAMLError as exc:
        logging.exception("Error parsing yaml")

    server = Server(config)
    server.run()
  
if __name__ == "__main__":
    main()