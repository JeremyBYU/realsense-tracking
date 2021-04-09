
import logging
import logging.handlers
import os
import time
import multiprocessing_logging

logger = None
def setup_logger(server=True):
    global logger
    LOG_FILENAME = 'logs/Python_Server_Log.log' if server else 'logs/Python_Client_Log.log'
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


    # Check if log exists and should therefore be rolled
    needRoll = os.path.isfile(LOG_FILENAME)
    # Add the log message handler to the logger
    fh = logging.handlers.RotatingFileHandler(LOG_FILENAME, backupCount=50)
    fh.setFormatter(formatter)
    logger.addHandler(fh)

    # This is a stale log, so roll it
    if needRoll:    
        # Add timestamp
        logger.debug('\n---------\nLog closed on %s.\n---------\n' % time.asctime())
        # Roll over on application start
        fh.doRollover()

    # Add timestamp
    logger.debug('\n---------\nLog started on %s.\n---------\n' % time.asctime())

    # multiprocessing_logging.install_mp_handler(logger)
    return logger