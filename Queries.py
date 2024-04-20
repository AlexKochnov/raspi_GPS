from abc import ABC
from UBXUtils import calc_checksum


class Query(ABC):
    # header = None
    msgId: hex
    classId: hex
    main_header = b'\xb5b'

    def __init__(self, msgId, classId):
        pass

    def serialize(self):
        pass

    @staticmethod
    def get():
        data = b''
        # cmd = self.classId.to_bytes() + self.msgId.to_bytes() + len(data).to_bytes(2, 'little') + data
        cmd = Query.classId.to_bytes() + Query.msgId.to_bytes() + len(data).to_bytes(2, 'little') + data
        return Query.main_header + cmd + calc_checksum(cmd)

    def set(self):
        pass

# class AID:
#     classId = 0x0B


class AID_ALM(Query):
    classId = 0x0B
    msgId = 0x31


class AID_EPH(Query):
    classId = 0x0B
    msgId = 0x30



# class AID_EPH(Query):
#     header = 0x0B, 0x31
#
#
# class AID_ALM(Query):
#     header = 0x0B, 0x30
