"""파라미터 편집 엔진 — 화이트리스트 스키마 + yaml round-trip 저장 (공용).

robot/sender 양쪽 런처가 공유한다. 스키마(ParamField 목록)는 각 측이 정의:
- launcher/robot/params.py  : ARM_SCHEMA / CAM_SCHEMA
- launcher/sender/params_schema.py : XR_DUAL_SCHEMA

동작:
- 저장은 ruamel.yaml round-trip (주석/순서 보존) → 임시 파일 → os.replace
  (atomic). ruamel 미설치 시 PyYAML fallback (comments_preserved=False).
- 모든 config 는 프로세스 시작 시 1회 로드되므로 저장 후 해당 컴포넌트
  재시작 필요 (restart_required=True).
- 검증은 all-or-nothing: 한 키라도 실패하면 아무것도 저장하지 않음.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

try:
    from ruamel.yaml import YAML
    _HAVE_RUAMEL = True
except ImportError:
    _HAVE_RUAMEL = False

import yaml as pyyaml


@dataclass
class ParamField:
    key: str                 # dotted path (예: safety.max_joint_vel)
    type: str                # float|int|str|bool|enum|range2|floats3|joints6|cam_list
    label: str
    unit: str = ""
    min: Optional[float] = None
    max: Optional[float] = None
    choices: Optional[List[str]] = None
    danger: bool = False     # 안전 관련 — UI 강조
    help: str = ""

    def to_dict(self) -> dict:
        return {k: v for k, v in self.__dict__.items() if v not in (None, "", False)} \
            | {"key": self.key, "type": self.type, "label": self.label,
               "danger": self.danger}


@dataclass
class SaveResult:
    ok: bool
    saved: List[str] = field(default_factory=list)
    errors: Dict[str, str] = field(default_factory=dict)
    comments_preserved: bool = True


# ── dotted-path helpers ──────────────────────────────────────────────────

def _get_dotted(data: dict, key: str):
    cur: Any = data
    for part in key.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return None
        cur = cur[part]
    return cur


def _set_dotted(data: dict, key: str, value) -> None:
    parts = key.split(".")
    cur = data
    for part in parts[:-1]:
        if part not in cur or not isinstance(cur[part], dict):
            cur[part] = {}
        cur = cur[part]
    cur[parts[-1]] = value


# ── validation ───────────────────────────────────────────────────────────

def _validate(f: ParamField, value) -> Tuple[Optional[Any], Optional[str]]:
    """(정규화된 값, 오류 메시지) 반환."""
    try:
        if f.type == "float":
            v = float(value)
        elif f.type == "int":
            v = int(value)
        elif f.type == "str":
            v = str(value)
            if not v.strip():
                return None, "빈 값"
        elif f.type == "bool":
            if isinstance(value, bool):
                v = value
            elif str(value).lower() in ("true", "1", "on", "yes"):
                v = True
            elif str(value).lower() in ("false", "0", "off", "no"):
                v = False
            else:
                return None, f"bool 아님: {value!r}"
        elif f.type == "enum":
            v = str(value)
            if v not in (f.choices or []):
                return None, f"{f.choices} 중 하나여야 함"
        elif f.type == "range2":
            v = [float(value[0]), float(value[1])]
            if len(value) != 2 or v[0] >= v[1]:
                return None, "[min, max] 형식 (min < max)"
        elif f.type == "floats3":
            if len(value) != 3:
                return None, "3개 값 필요"
            v = [float(x) for x in value]
        elif f.type == "joints6":
            if len(value) != 6:
                return None, "6개 값 필요"
            v = [float(x) for x in value]
        elif f.type == "cam_list":
            if not isinstance(value, list) or not value:
                return None, "1개 이상의 카메라 행 필요"
            v = []
            names = set()
            for i, row in enumerate(value):
                name = str(row.get("name", "")).strip()
                if not name:
                    return None, f"{i+1}번째 행: name 필요"
                if name in names:
                    return None, f"카메라 이름 중복: {name}"
                names.add(name)
                w, h, fps = int(row.get("width", 640)), int(row.get("height", 480)), \
                    int(row.get("fps", 30))
                if not (64 <= w <= 4096 and 64 <= h <= 4096 and 1 <= fps <= 120):
                    return None, f"{name}: width/height/fps 범위 오류"
                v.append({"name": name, "serial": str(row.get("serial", "")),
                          "width": w, "height": h, "fps": fps})
        else:
            return None, f"알 수 없는 type {f.type}"
    except (TypeError, ValueError, KeyError, IndexError) as e:
        return None, f"형식 오류: {e}"

    if f.type in ("float", "int"):
        if f.min is not None and v < f.min:
            return None, f"최소 {f.min}"
        if f.max is not None and v > f.max:
            return None, f"최대 {f.max}"
    return v, None


# ── public API ───────────────────────────────────────────────────────────

def get_params(file_path: str, schema: List[ParamField]) -> dict:
    data = pyyaml.safe_load(Path(file_path).read_text()) or {}
    values = {f.key: _get_dotted(data, f.key) for f in schema}
    return {
        "schema": [f.to_dict() for f in schema],
        "values": values,
        "file": file_path,
        "restart_required": True,
        "comments_preserved": _HAVE_RUAMEL,
    }


def set_params(file_path: str, schema: List[ParamField],
               values: Dict[str, Any]) -> SaveResult:
    by_key = {f.key: f for f in schema}
    result = SaveResult(ok=False, comments_preserved=_HAVE_RUAMEL)

    # 1) 화이트리스트 + 검증 (all-or-nothing)
    normalized: Dict[str, Any] = {}
    for key, raw in values.items():
        f = by_key.get(key)
        if f is None:
            result.errors[key] = "허용되지 않은 파라미터"
            continue
        v, err = _validate(f, raw)
        if err:
            result.errors[key] = err
        else:
            normalized[key] = v
    if result.errors or not normalized:
        if not normalized and not result.errors:
            result.errors["_"] = "변경할 값 없음"
        return result

    # 2) round-trip 로드 → 수정 → atomic 저장
    path = Path(file_path)
    if _HAVE_RUAMEL:
        ry = YAML()   # round-trip 모드 (주석/순서 보존)
        ry.preserve_quotes = True
        with open(path) as fh:
            data = ry.load(fh) or {}
        for key, v in normalized.items():
            _set_dotted(data, key, v)
        tmp = path.with_suffix(path.suffix + ".tmp")
        with open(tmp, "w") as fh:
            ry.dump(data, fh)
    else:
        data = pyyaml.safe_load(path.read_text()) or {}
        for key, v in normalized.items():
            _set_dotted(data, key, v)
        tmp = path.with_suffix(path.suffix + ".tmp")
        tmp.write_text(pyyaml.safe_dump(data, allow_unicode=True,
                                        default_flow_style=False, sort_keys=False))
    os.replace(tmp, path)

    result.ok = True
    result.saved = sorted(normalized.keys())
    return result
