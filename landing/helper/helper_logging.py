
import logging

FORMAT = "%(message)s"
logging.basicConfig(
    level=logging.INFO, format=FORMAT, datefmt="[%X]"
)

logger = logging.getLogger("RealSenseLanding")
logger.setLevel(logging.INFO)