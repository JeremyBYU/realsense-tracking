
import logging
import logging.handlers
from logging import StreamHandler
import os
import time
import multiprocessing_logging
import sys

logger = None

class TUIHandler(StreamHandler):
    def __init__(self, text_box):
        StreamHandler.__init__(self)
        self.current_text = ""
        self.text_box = text_box
        self.y_start = 0

    def emit(self, record):
        msg = self.format(record)

        num_lines_before = len(self.text_box._text_lines)
        new_lines = msg.split('\n')
        self.text_box._text_lines.extend(new_lines)
        num_lines_after = len(self.text_box._text_lines)
        lines_added = num_lines_after - num_lines_before
        start_y = self.text_box._viewport_height - num_lines_after

        if start_y < 0:
            self.y_start = self.y_start + lines_added
            self.text_box._viewport_y_start = self.y_start

def add_tui_handler(tui):
    global logger
    th = TUIHandler(tui)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    th.setLevel(logging.INFO)
    th.setFormatter(formatter)
    logger.addHandler(th)
    return logger


def setup_logger(server=True, add_stream_handler=True):
    global logger
    LOG_FILENAME = 'logs/Python_Server_Log.log' if server else 'logs/Python_Client_Log.log'
    FORMAT = "%(message)s"
    logger = logging.getLogger("RealSenseLanding")
    logger.setLevel(logging.INFO)
    logger.propagate = False
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

    if add_stream_handler:
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)
        # create formatter and add it to the handlers
        ch.setFormatter(formatter)
        # add the handlers to the logger
        logger.addHandler(ch)

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