from typing import List, Callable

from flask import Request, Response, jsonify

from http_io.http_receiver import HttpPathHandler

from http_io.mission_model import JsonMissionPoint, JsonMissionsMessage, JsonMissionPackage

from pathlib import Path
import os
import glob


class MissionExtractor:
    MISSION_FILE_DESTINATION = str(Path.home()) + "/missions/"
    BASIC_MISSION_NAME = 'mission_region{0}_route{1}_variant{2}.waypoint'
    MISSION_HEADER = 'QGC WPL 110\n'

    @staticmethod
    def extract_point_entity(point_entity: any) -> str:
        return '{0}\t'.format(point_entity)

    @staticmethod
    def replace_suffix(point_raw: str) -> str:
        return point_raw[:-1] + '\n'

    def extract_point(self, mission_point: JsonMissionPoint) -> str:
        point_raw = ''
        for key, value in mission_point.dict().items():
            point_raw = point_raw + self.extract_point_entity(value)
        return self.replace_suffix(point_raw)

    @staticmethod
    def get_file_path(file_name: str) -> str:
        return MissionExtractor.MISSION_FILE_DESTINATION + file_name

    @staticmethod
    def save_mission_to_file(mission_raw: str, file_name: str) -> str:

        file_path = MissionExtractor.get_file_path(file_name)

        with open(file_path, 'w') as file:
            file.write(mission_raw)

        return file_path

    def extract_mission_points(self, mission_points: List[JsonMissionPoint], file_name: str) -> str:
        mission_raw: str = MissionExtractor.MISSION_HEADER
        for point in mission_points:
            mission_raw = mission_raw + self.extract_point(point)

        return self.save_mission_to_file(mission_raw, file_name)

    @staticmethod
    def get_file_name(mission: JsonMissionPackage, operating_body: JsonMissionsMessage) -> str:
        return MissionExtractor.BASIC_MISSION_NAME.format(operating_body.region_id,
                                                          mission.route_id,
                                                          mission.route_variant)

    def extract_files(self, operating_body: JsonMissionsMessage) -> str:
        self.clear_dir()
        missions = []
        for mission in operating_body.missions:
            missions.append(self.extract_mission_points(mission.mission_points,
                                                        self.get_file_name(mission, operating_body)))
        return missions[0]

    def clear_dir(self):
        files = glob.glob(self.MISSION_FILE_DESTINATION + '/*')
        for f in files:
            os.remove(f)


class MissionReceiver(HttpPathHandler):

    def __init__(self, task_handler: Callable[[str], bool]):
        HttpPathHandler.__init__(self)
        self.mission_extractor = MissionExtractor()
        self._task_handler = task_handler

    def process_request(self, incoming_request: Request):
        mission_name = self.mission_extractor.extract_files(JsonMissionsMessage.parse_obj(incoming_request.json))

        result = {"is_ok": False}

        if self._task_handler(mission_name):
            result["is_ok"] = True

        return jsonify(result)
