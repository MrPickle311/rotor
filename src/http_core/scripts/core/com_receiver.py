from http_io.http_receiver import HttpPathHandler, ServerAPI
from flask import request


class COMServerAPI(ServerAPI):
    def endpoints_creation(self):
        @self.route('/com/kss_event/<string:kss_event>', methods=['POST'])
        def kss_event(kss_event: str):
            return self._route_handler(request)
