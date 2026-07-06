from __future__ import annotations


class GraspFailure(Exception):
    def __init__(self, error_code: int, failed_stage: str, message: str):
        super().__init__(message)
        self.error_code = int(error_code)
        self.failed_stage = str(failed_stage)


class GraspCancelled(Exception):
    pass
