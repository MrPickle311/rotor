from typing import Callable

from flask import Request, jsonify

from http_io.http_receiver import HttpPathHandler


class KSSTaskReceiver(HttpPathHandler):

    def __init__(self, task_handler: Callable[[], bool]):
        HttpPathHandler.__init__(self)
        self._task_handler = task_handler

    def process_request(self, incoming_request: Request):
        result = {"is_ok": False}

        if self._task_handler():
            result["is_ok"] = True

        return jsonify(result)
