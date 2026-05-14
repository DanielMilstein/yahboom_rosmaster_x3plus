#!/usr/bin/env python3

import hashlib
import json
import os
import struct
import time
import zlib
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from yahboom_rosmaster_msgs.srv import GeminiPickPlace, GeminiVerifyPick


SCHEMA_VERSION = "pick_place_v1"
VALID_STATUSES = {"ready", "not_found", "ambiguous", "blocked", "unsafe"}
VALID_STEPS = {
    "locate_target",
    "locate_destination",
    "approach_target",
    "grasp_target",
    "lift",
    "move_to_destination",
    "release",
    "retreat",
}
PROHIBITED_KEY_FRAGMENTS = (
    "trajectory",
    "joint",
    "velocity",
    "controller",
    "command",
    "twist",
)


PICK_PLACE_SCHEMA = {
    "type": "object",
    "additionalProperties": False,
    "required": [
        "schema_version",
        "status",
        "confidence",
        "summary",
        "target_object",
        "destination",
        "scene_checks",
        "high_level_steps",
        "constraints",
    ],
    "properties": {
        "schema_version": {"type": "string", "enum": [SCHEMA_VERSION]},
        "status": {"type": "string", "enum": sorted(VALID_STATUSES)},
        "confidence": {"type": "number", "minimum": 0.0, "maximum": 1.0},
        "summary": {"type": "string", "maxLength": 240},
        "target_object": {
            "type": "object",
            "additionalProperties": False,
            "required": ["label", "point", "confidence", "visibility_notes"],
            "properties": {
                "label": {"type": "string", "maxLength": 80},
                "point": {
                    "type": "array",
                    "items": {"type": "number", "minimum": 0.0, "maximum": 1.0},
                    "minItems": 2,
                    "maxItems": 2,
                },
                "box": {
                    "type": "array",
                    "items": {"type": "number", "minimum": 0.0, "maximum": 1.0},
                    "minItems": 4,
                    "maxItems": 4,
                },
                "confidence": {"type": "number", "minimum": 0.0, "maximum": 1.0},
                "visibility_notes": {"type": "string", "maxLength": 200},
            },
        },
        "destination": {
            "type": "object",
            "additionalProperties": False,
            "required": ["label", "point", "confidence"],
            "properties": {
                "label": {"type": "string", "maxLength": 80},
                "point": {
                    "type": "array",
                    "items": {"type": "number", "minimum": 0.0, "maximum": 1.0},
                    "minItems": 2,
                    "maxItems": 2,
                },
                "box": {
                    "type": "array",
                    "items": {"type": "number", "minimum": 0.0, "maximum": 1.0},
                    "minItems": 4,
                    "maxItems": 4,
                },
                "confidence": {"type": "number", "minimum": 0.0, "maximum": 1.0},
            },
        },
        "scene_checks": {
            "type": "object",
            "additionalProperties": False,
            "required": [
                "target_visible",
                "destination_visible",
                "grasp_appears_possible",
                "placement_appears_possible",
            ],
            "properties": {
                "target_visible": {"type": "boolean"},
                "destination_visible": {"type": "boolean"},
                "grasp_appears_possible": {"type": "boolean"},
                "placement_appears_possible": {"type": "boolean"},
            },
        },
        "high_level_steps": {
            "type": "array",
            "items": {"type": "string", "enum": sorted(VALID_STEPS)},
            "minItems": 1,
            "maxItems": 8,
        },
        "constraints": {
            "type": "array",
            "items": {"type": "string", "maxLength": 120},
            "maxItems": 8,
        },
    },
}


def utc_stamp():
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")


def read_prompt(filename):
    share_dir = Path(get_package_share_directory("gemini_robotics_bridge"))
    return (share_dir / "prompts" / filename).read_text(encoding="utf-8").strip()


def png_chunk(chunk_type, data):
    crc = zlib.crc32(chunk_type)
    crc = zlib.crc32(data, crc) & 0xFFFFFFFF
    return (
        struct.pack(">I", len(data))
        + chunk_type
        + data
        + struct.pack(">I", crc)
    )


def encode_png(width, height, color_type, rows):
    header = struct.pack(">IIBBBBB", width, height, 8, color_type, 0, 0, 0)
    raw = b"".join(b"\x00" + row for row in rows)
    return (
        b"\x89PNG\r\n\x1a\n"
        + png_chunk(b"IHDR", header)
        + png_chunk(b"IDAT", zlib.compress(raw))
        + png_chunk(b"IEND", b"")
    )


def image_msg_to_png_bytes(image_msg):
    width = int(image_msg.width)
    height = int(image_msg.height)
    step = int(image_msg.step)
    encoding = image_msg.encoding.lower()
    data = bytes(image_msg.data)

    if width <= 0 or height <= 0:
        raise RuntimeError("request image has invalid dimensions")

    if encoding in ("rgb8", "bgr8"):
        rows = []
        for y in range(height):
            source = data[y * step : y * step + width * 3]
            if len(source) < width * 3:
                raise RuntimeError("request image data is shorter than expected")
            if encoding == "bgr8":
                source = b"".join(
                    source[i + 2 : i + 3] + source[i + 1 : i + 2] + source[i : i + 1]
                    for i in range(0, width * 3, 3)
                )
            rows.append(source)
        return encode_png(width, height, 2, rows)

    if encoding in ("rgba8", "bgra8"):
        rows = []
        for y in range(height):
            source = data[y * step : y * step + width * 4]
            if len(source) < width * 4:
                raise RuntimeError("request image data is shorter than expected")
            if encoding == "bgra8":
                source = b"".join(
                    source[i + 2 : i + 3]
                    + source[i + 1 : i + 2]
                    + source[i : i + 1]
                    + source[i + 3 : i + 4]
                    for i in range(0, width * 4, 4)
                )
            rows.append(source)
        return encode_png(width, height, 6, rows)

    if encoding in ("mono8", "8uc1"):
        rows = []
        for y in range(height):
            source = data[y * step : y * step + width]
            if len(source) < width:
                raise RuntimeError("request image data is shorter than expected")
            rows.append(source)
        return encode_png(width, height, 0, rows)

    raise RuntimeError(
        "unsupported image encoding "
        f"{image_msg.encoding!r}; supported encodings are rgb8, bgr8, rgba8, "
        "bgra8, mono8, and 8UC1"
    )


def text_from_response(response):
    text = getattr(response, "text", None)
    if text:
        return text

    chunks = []
    for candidate in getattr(response, "candidates", []) or []:
        content = getattr(candidate, "content", None)
        for part in getattr(content, "parts", []) or []:
            part_text = getattr(part, "text", None)
            if part_text:
                chunks.append(part_text)
    return "\n".join(chunks)


def validate_pick_place_result(data):
    errors = []

    if not isinstance(data, dict):
        return ["top-level JSON value must be an object"]

    allowed_top = set(PICK_PLACE_SCHEMA["properties"])
    reject_extra_keys(data, allowed_top, "$", errors)
    require_keys(data, PICK_PLACE_SCHEMA["required"], "$", errors)
    reject_prohibited_keys(data, "$", errors)

    if data.get("schema_version") != SCHEMA_VERSION:
        errors.append("$.schema_version must be pick_place_v1")
    if data.get("status") not in VALID_STATUSES:
        errors.append(f"$.status must be one of {sorted(VALID_STATUSES)}")
    check_number(data.get("confidence"), "$.confidence", errors)
    check_string(data.get("summary"), "$.summary", errors, max_len=240)

    validate_entity(
        data.get("target_object"),
        "$.target_object",
        {"label", "point", "box", "confidence", "visibility_notes"},
        ["label", "point", "confidence", "visibility_notes"],
        errors,
        require_visibility_notes=True,
    )
    validate_entity(
        data.get("destination"),
        "$.destination",
        {"label", "point", "box", "confidence"},
        ["label", "point", "confidence"],
        errors,
    )
    validate_scene_checks(data.get("scene_checks"), errors)
    validate_steps(data.get("high_level_steps"), errors)
    validate_constraints(data.get("constraints"), errors)

    return errors


def schema_for_gemini_api(schema):
    api_schema = deepcopy(schema)
    strip_schema_key(api_schema, "additionalProperties")
    return api_schema


def strip_schema_key(value, key_to_strip):
    if isinstance(value, dict):
        value.pop(key_to_strip, None)
        for child in value.values():
            strip_schema_key(child, key_to_strip)
    elif isinstance(value, list):
        for child in value:
            strip_schema_key(child, key_to_strip)


def reject_extra_keys(obj, allowed, path, errors):
    if isinstance(obj, dict):
        for key in obj:
            if key not in allowed:
                errors.append(f"{path}.{key} is not allowed")


def reject_prohibited_keys(value, path, errors):
    if isinstance(value, dict):
        for key, child in value.items():
            lower = key.lower()
            if any(fragment in lower for fragment in PROHIBITED_KEY_FRAGMENTS):
                errors.append(f"{path}.{key} is prohibited")
            reject_prohibited_keys(child, f"{path}.{key}", errors)
    elif isinstance(value, list):
        for index, child in enumerate(value):
            reject_prohibited_keys(child, f"{path}[{index}]", errors)


def require_keys(obj, keys, path, errors):
    if not isinstance(obj, dict):
        errors.append(f"{path} must be an object")
        return
    for key in keys:
        if key not in obj:
            errors.append(f"{path}.{key} is required")


def check_string(value, path, errors, max_len=None):
    if not isinstance(value, str):
        errors.append(f"{path} must be a string")
        return
    if max_len is not None and len(value) > max_len:
        errors.append(f"{path} must be at most {max_len} characters")


def check_number(value, path, errors):
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        errors.append(f"{path} must be a number")
        return
    if not 0.0 <= float(value) <= 1.0:
        errors.append(f"{path} must be between 0.0 and 1.0")


def check_point(value, path, errors):
    if not isinstance(value, list) or len(value) != 2:
        errors.append(f"{path} must be [y, x]")
        return
    for index, coordinate in enumerate(value):
        check_number(coordinate, f"{path}[{index}]", errors)


def check_box(value, path, errors):
    if value is None:
        return
    if not isinstance(value, list) or len(value) != 4:
        errors.append(f"{path} must be [ymin, xmin, ymax, xmax]")
        return
    for index, coordinate in enumerate(value):
        check_number(coordinate, f"{path}[{index}]", errors)
    if len(value) == 4 and all(isinstance(v, (int, float)) for v in value):
        if value[0] > value[2] or value[1] > value[3]:
            errors.append(f"{path} min coordinates must not exceed max coordinates")


def validate_entity(value, path, allowed, required, errors, require_visibility_notes=False):
    if not isinstance(value, dict):
        errors.append(f"{path} must be an object")
        return
    reject_extra_keys(value, allowed, path, errors)
    require_keys(value, required, path, errors)
    check_string(value.get("label"), f"{path}.label", errors, max_len=80)
    check_point(value.get("point"), f"{path}.point", errors)
    if "box" in value:
        check_box(value.get("box"), f"{path}.box", errors)
    check_number(value.get("confidence"), f"{path}.confidence", errors)
    if require_visibility_notes:
        check_string(
            value.get("visibility_notes"),
            f"{path}.visibility_notes",
            errors,
            max_len=200,
        )


def validate_scene_checks(value, errors):
    path = "$.scene_checks"
    required = [
        "target_visible",
        "destination_visible",
        "grasp_appears_possible",
        "placement_appears_possible",
    ]
    if not isinstance(value, dict):
        errors.append(f"{path} must be an object")
        return
    reject_extra_keys(value, set(required), path, errors)
    require_keys(value, required, path, errors)
    for key in required:
        if not isinstance(value.get(key), bool):
            errors.append(f"{path}.{key} must be a boolean")


def validate_steps(value, errors):
    path = "$.high_level_steps"
    if not isinstance(value, list) or not value:
        errors.append(f"{path} must be a non-empty list")
        return
    if len(value) > 8:
        errors.append(f"{path} must contain at most 8 steps")
    for index, step in enumerate(value):
        if step not in VALID_STEPS:
            errors.append(f"{path}[{index}] must be one of {sorted(VALID_STEPS)}")


def validate_constraints(value, errors):
    path = "$.constraints"
    if not isinstance(value, list):
        errors.append(f"{path} must be a list")
        return
    if len(value) > 8:
        errors.append(f"{path} must contain at most 8 constraints")
    for index, constraint in enumerate(value):
        if not isinstance(constraint, str):
            errors.append(f"{path}[{index}] must be a string")
        elif len(constraint) > 120:
            errors.append(f"{path}[{index}] must be at most 120 characters")


class GeminiRoboticsBridge(Node):
    def __init__(self):
        super().__init__("gemini_robotics_bridge")

        self.declare_parameter("api_key_env", "GEMINI_API_KEY")
        self.declare_parameter(
            "api_key_envs",
            ["GEMINI_API_KEY", "GEMINI_API_KEY_2", "GEMINI_API_KEY_3"],
        )
        self.declare_parameter("model_name", "gemini-robotics-er-1.6-preview")
        self.declare_parameter("request_timeout_sec", 60.0)
        self.declare_parameter("max_retries", 2)
        self.declare_parameter("retry_backoff_sec", 1.0)
        self.declare_parameter("confidence_threshold", 0.65)
        self.declare_parameter("log_dir", "~/.ros/gemini_robotics_bridge")
        self.declare_parameter("temperature", 0.1)
        self.declare_parameter("thinking_budget", 0)

        self.api_key_env = self.get_parameter("api_key_env").value

        env_list_param = list(self.get_parameter("api_key_envs").value)
        env_names = [name for name in env_list_param if name] or [self.api_key_env]

        api_keys, env_names_used = [], []
        for name in env_names:
            val = os.environ.get(name)
            if val:
                api_keys.append(val)
                env_names_used.append(name)
            else:
                self.get_logger().info(f"{name} not set; skipping")

        if not api_keys:
            raise RuntimeError(
                f"none of {env_names} are set; export at least one API key before "
                "starting gemini_robotics_bridge."
            )

        try:
            from google import genai
        except ImportError as exc:
            raise RuntimeError(
                "google-genai is not installed. Install it with "
                "`python3 -m pip install -r "
                "src/yahboom_rosmaster/gemini_robotics_bridge/requirements-gemini.txt`."
            ) from exc

        self.api_keys = api_keys
        self.api_key_envs_used = env_names_used
        self.active_key_index = 0
        self.genai = genai
        self.client = self.create_genai_client(genai, self.api_keys[0])
        self.get_logger().info(
            f"Loaded {len(self.api_keys)} API key(s) from {self.api_key_envs_used}; "
            f"starting with {self.api_key_envs_used[0]}"
        )
        self.system_prompt = read_prompt("gemini_pick_place_system.txt")
        self.retry_prompt = read_prompt("gemini_pick_place_retry.txt")
        self.verify_prompt = read_prompt("gemini_verify_pick.txt")
        self.log_dir = Path(str(self.get_parameter("log_dir").value)).expanduser()
        self.log_dir.mkdir(parents=True, exist_ok=True)

        self.service = self.create_service(
            GeminiPickPlace, "gemini_pick_place", self.handle_pick_place
        )
        self.verify_service = self.create_service(
            GeminiVerifyPick, "gemini_verify_pick", self.handle_verify_pick
        )
        self.get_logger().info(
            "Gemini Robotics bridge ready on /gemini_pick_place and /gemini_verify_pick"
        )

    def _is_quota_or_auth_error(self, exc):
        msg = str(exc).lower()
        indicators = (
            "quota", "rate limit", "rate_limit", "429",
            "resource_exhausted", "resource exhausted",
            "permission_denied", "permission denied",
            "unauthorized", "401", "403", "forbidden",
        )
        return any(ind in msg for ind in indicators)

    def _rotate_api_key(self):
        self.active_key_index = (self.active_key_index + 1) % len(self.api_keys)
        next_env = self.api_key_envs_used[self.active_key_index]
        self.get_logger().warn(
            f"rotating to API key #{self.active_key_index + 1} ({next_env})"
        )
        self.client = self.create_genai_client(
            self.genai, self.api_keys[self.active_key_index]
        )

    def create_genai_client(self, genai, api_key):
        timeout_ms = int(float(self.get_parameter("request_timeout_sec").value) * 1000.0)
        try:
            from google.genai import types

            return genai.Client(
                api_key=api_key,
                http_options=types.HttpOptions(timeout=timeout_ms),
            )
        except Exception:
            self.get_logger().warn(
                "google-genai client did not accept request_timeout_sec; "
                "using SDK default timeout"
            )
            return genai.Client(api_key=api_key)

    def handle_pick_place(self, request, response):
        request_stamp = utc_stamp()
        request_dir = self.log_dir / request_stamp
        request_dir.mkdir(parents=True, exist_ok=True)

        try:
            image_bytes, image_hash, image_path = self.encode_image(request.image, request_dir)
            result, raw_text, attempts = self.query_with_retries(
                request.task, image_bytes, image_hash, request_dir
            )
        except Exception as exc:
            error_message = str(exc)
            self.write_json(
                request_dir / "failure.json",
                {
                    "timestamp_utc": request_stamp,
                    "task": request.task,
                    "error": error_message,
                    "api_key_env": self.api_key_env,
                },
            )
            response.success = False
            response.accepted = False
            response.confidence = 0.0
            response.result_json = ""
            response.error_message = error_message
            response.log_path = str(request_dir)
            self.get_logger().error(error_message)
            return response

        accepted, error_message = self.acceptance_decision(result)
        result_json = json.dumps(result, sort_keys=True, separators=(",", ":"))

        self.write_json(
            request_dir / "request_response.json",
            {
                "timestamp_utc": request_stamp,
                "task": request.task,
                "model_name": self.get_parameter("model_name").value,
                "confidence_threshold": float(
                    self.get_parameter("confidence_threshold").value
                ),
                "image_hash_sha256": image_hash,
                "image_path": str(image_path),
                "system_prompt": self.system_prompt,
                "retry_prompt": self.retry_prompt,
                "schema": PICK_PLACE_SCHEMA,
                "attempts": attempts,
                "final_raw_response": raw_text,
                "parsed_json": result,
                "accepted": accepted,
                "error_message": error_message,
            },
        )

        response.success = True
        response.accepted = accepted
        response.confidence = float(result.get("confidence", 0.0))
        response.result_json = result_json
        response.error_message = error_message
        response.log_path = str(request_dir)
        return response

    def handle_verify_pick(self, request, response):
        request_stamp = utc_stamp()
        request_dir = self.log_dir / f"verify_{request_stamp}"
        request_dir.mkdir(parents=True, exist_ok=True)
        try:
            image_bytes, image_hash, image_path = self.encode_image(
                request.image, request_dir
            )
            prompt = (
                f"{self.verify_prompt}\n\n"
                f"Target object: {request.target_label}\n"
            )
            raw_text = self.call_model(prompt, image_bytes)
            parsed = None
            error_message = ""
            try:
                parsed = json.loads(raw_text)
            except json.JSONDecodeError as exc:
                error_message = f"invalid JSON from Gemini: {exc}"
            self.write_json(
                request_dir / "request_response.json",
                {
                    "timestamp_utc": request_stamp,
                    "target_label": request.target_label,
                    "image_hash_sha256": image_hash,
                    "image_path": str(image_path),
                    "verify_prompt": self.verify_prompt,
                    "raw_response": raw_text,
                    "parsed_json": parsed,
                    "error_message": error_message,
                },
            )
        except Exception as exc:
            response.success = False
            response.picked_up = False
            response.confidence = 0.0
            response.reason = ""
            response.log_path = str(request_dir)
            response.error_message = str(exc)
            self.get_logger().error(f"verify_pick failed: {exc}")
            return response

        if parsed is None:
            response.success = False
            response.picked_up = False
            response.confidence = 0.0
            response.reason = ""
            response.log_path = str(request_dir)
            response.error_message = error_message
            return response

        response.success = True
        response.picked_up = bool(parsed.get("picked_up", False))
        try:
            response.confidence = float(parsed.get("confidence", 0.0))
        except (TypeError, ValueError):
            response.confidence = 0.0
        response.reason = str(parsed.get("reason", ""))
        response.log_path = str(request_dir)
        response.error_message = ""
        return response

    def encode_image(self, image_msg, request_dir):
        image_bytes = image_msg_to_png_bytes(image_msg)
        image_hash = hashlib.sha256(image_bytes).hexdigest()
        image_path = request_dir / f"scene_{image_hash[:16]}.png"
        image_path.write_bytes(image_bytes)
        return image_bytes, image_hash, image_path

    def query_with_retries(self, task, image_bytes, image_hash, request_dir):
        max_retries = int(self.get_parameter("max_retries").value)
        attempts = []
        previous_error = ""
        previous_raw = ""

        for attempt_index in range(max_retries + 1):
            prompt = self.build_prompt(task, attempt_index, previous_error, previous_raw)
            raw_text = self.call_model(prompt, image_bytes)
            attempt_log_path = request_dir / f"attempt_{attempt_index + 1}.json"

            parsed = None
            errors = []
            try:
                parsed = json.loads(raw_text)
                errors = validate_pick_place_result(parsed)
            except json.JSONDecodeError as exc:
                errors = [f"invalid JSON: {exc}"]

            attempt = {
                "attempt": attempt_index + 1,
                "image_hash_sha256": image_hash,
                "prompt": prompt,
                "raw_response": raw_text,
                "parsed_json": parsed,
                "validation_errors": errors,
            }
            attempts.append(attempt)
            self.write_json(attempt_log_path, attempt)

            if not errors and parsed is not None:
                return parsed, raw_text, attempts

            previous_error = "; ".join(errors)
            previous_raw = raw_text
            if attempt_index < max_retries:
                time.sleep(float(self.get_parameter("retry_backoff_sec").value))

        raise RuntimeError(
            "Gemini response did not match pick_place_v1 schema after "
            f"{max_retries + 1} attempt(s): {previous_error}"
        )

    def build_prompt(self, task, attempt_index, previous_error, previous_raw):
        prompt = (
            f"{self.system_prompt}\n\n"
            "Task:\n"
            f"{task}\n\n"
            "Required JSON schema:\n"
            f"{json.dumps(PICK_PLACE_SCHEMA, sort_keys=True)}"
        )
        if attempt_index == 0:
            return prompt
        return (
            f"{self.retry_prompt}\n\n"
            f"Original task:\n{task}\n\n"
            f"Previous validation error:\n{previous_error}\n\n"
            f"Previous raw response:\n{previous_raw}\n\n"
            "Required JSON schema:\n"
            f"{json.dumps(PICK_PLACE_SCHEMA, sort_keys=True)}"
        )

    def call_model(self, prompt, image_bytes):
        try:
            from google.genai import types
        except ImportError:
            types = None

        if types is not None:
            image_part = types.Part.from_bytes(data=image_bytes, mime_type="image/png")
            contents = [prompt, image_part]
        else:
            contents = [
                prompt,
                {"inline_data": {"mime_type": "image/png", "data": image_bytes}},
            ]

        config = {
            "response_mime_type": "application/json",
            "response_schema": schema_for_gemini_api(PICK_PLACE_SCHEMA),
            "temperature": float(self.get_parameter("temperature").value),
        }
        thinking_budget = int(self.get_parameter("thinking_budget").value)
        if thinking_budget >= 0:
            config["thinking_config"] = {"thinking_budget": thinking_budget}

        model_name = self.get_parameter("model_name").value

        last_exception = None
        keys_tried = 0
        while keys_tried < len(self.api_keys):
            try:
                try:
                    response = self.client.models.generate_content(
                        model=model_name,
                        contents=contents,
                        config=config,
                    )
                except TypeError:
                    config.pop("thinking_config", None)
                    response = self.client.models.generate_content(
                        model=model_name,
                        contents=contents,
                        config=config,
                    )
                raw_text = text_from_response(response).strip()
                if not raw_text:
                    raise RuntimeError("Gemini response contained no text")
                return raw_text
            except Exception as exc:
                last_exception = exc
                if not self._is_quota_or_auth_error(exc):
                    raise
                keys_tried += 1
                if keys_tried >= len(self.api_keys):
                    break
                self.get_logger().warn(
                    f"key #{self.active_key_index + 1} "
                    f"({self.api_key_envs_used[self.active_key_index]}) "
                    f"hit quota/auth error: {exc}; rotating"
                )
                self._rotate_api_key()

        raise RuntimeError(
            f"all {len(self.api_keys)} API key(s) exhausted on quota/auth errors; "
            f"last error: {last_exception}"
        )

    def acceptance_decision(self, result):
        status = result.get("status")
        confidence = float(result.get("confidence", 0.0))
        threshold = float(self.get_parameter("confidence_threshold").value)

        if status == "unsafe":
            return False, "model marked the scene or task unsafe"
        if status != "ready":
            return False, f"model status is {status}"
        if confidence < threshold:
            return False, (
                f"confidence {confidence:.3f} is below threshold {threshold:.3f}"
            )
        return True, ""

    def write_json(self, path, data):
        path.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")


def main(args=None):
    rclpy.init(args=args)
    node = GeminiRoboticsBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
