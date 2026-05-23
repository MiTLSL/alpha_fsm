def build_request(failure_name: str, duration_sec: float = 0.0, params_json: str = "{}"):
    return {
        "failure_name": failure_name,
        "duration_sec": duration_sec,
        "params_json": params_json,
    }
