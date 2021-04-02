
import logging
import multiprocessing_logging

FORMAT = "%(message)s"

logger = logging.getLogger("RealSenseLanding")
logger.setLevel(logging.INFO)

ch = logging.StreamHandler()
ch.setLevel(logging.INFO)
# create formatter and add it to the handlers
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
ch.setFormatter(formatter)
# add the handlers to the logger
logger.addHandler(ch)
logger.propagate = False


# multiprocessing_logging.install_mp_handler(logger)