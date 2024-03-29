import sys
import logging


def build_logger(name: str, console_output=True, **kargs):
    if console_output and "filename" in kargs:
        handle = [
            logging.FileHandler(kargs["filename"]),
            logging.StreamHandler(sys.stdout),
        ]
        kargs["handlers"] = handle
        del kargs["filename"]

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        **kargs
    )
    return logging.getLogger(name)


def list_to_str(ids: list):
    string = ""
    for id in ids:
        string += str(id) + ","

    return string.strip(",")
