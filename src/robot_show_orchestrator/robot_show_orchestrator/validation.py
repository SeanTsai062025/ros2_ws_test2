"""Strict validation for Robot Show Orchestrator YAML files."""

from __future__ import annotations

from copy import deepcopy
import math
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, MutableMapping, Set

import yaml


FACE_TYPES = {
    'DEFAULT',
    'HAPPY',
    'ANGRY',
    'TIRED',
    'LOOK LEFT',
    'LOOK RIGHT',
    'LOOK UP',
    'LOOK DOWN',
    'LOOK CENTER',
    'BLINK',
    'WINK LEFT',
    'WINK RIGHT',
    'LAUGH',
    'CONFUSED',
    'IDLE ON',
    'IDLE OFF',
}
TARGETS = {'arm', 'rail1', 'rail2', 'servo1', 'servo2', 'screen'}
RAILS = {'rail1', 'rail2'}


class ShowValidationError(ValueError):
    """Raised when a show cannot be executed safely."""

    def __init__(self, issues: Iterable[str]):
        self.issues = list(issues)
        super().__init__('; '.join(self.issues))


class LocatedMapping(dict):
    """YAML mapping that remembers where it was written."""

    def __init__(
        self,
        *args,
        source_file: str = '',
        source_line: int = 0,
        source_column: int = 0,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.source_file = source_file
        self.source_line = source_line
        self.source_column = source_column


class _LocatedSafeLoader(yaml.SafeLoader):
    """Safe YAML loader that attaches file and line metadata to mappings."""

    def __init__(self, stream, source_file: str):
        super().__init__(stream)
        self.source_file = source_file


def _construct_located_mapping(loader: _LocatedSafeLoader, node):
    mapping = LocatedMapping(
        source_file=loader.source_file,
        source_line=node.start_mark.line + 1,
        source_column=node.start_mark.column + 1,
    )
    yield mapping
    mapping.update(loader.construct_mapping(node))


_LocatedSafeLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_located_mapping,
)


def source_reference(value: Any, yaml_path: str = '') -> str:
    """Return a human-readable source location for a loaded YAML mapping."""
    source_file = getattr(value, 'source_file', '')
    source_line = getattr(value, 'source_line', 0)
    source_column = getattr(value, 'source_column', 0)
    if source_file and source_line:
        location = f'{source_file}:{source_line}'
        if source_column and source_column != 1:
            location += f':{source_column}'
        return f'{location} ({yaml_path})' if yaml_path else location
    return yaml_path


def load_show(path: str) -> Dict[str, Any]:
    """Load and validate one YAML show file."""
    show_path = Path(path).expanduser().resolve()
    try:
        with show_path.open('r', encoding='utf-8') as stream:
            loader = _LocatedSafeLoader(stream, str(show_path))
            try:
                document = loader.get_single_data()
            finally:
                loader.dispose()
    except (OSError, yaml.YAMLError) as error:
        raise ShowValidationError([f'cannot load {show_path}: {error}']) from error
    validate_show(document)
    return document


def _is_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _require_number(
    value: Any, path: str, issues: list[str], minimum: float | None = None
) -> bool:
    if not _is_number(value):
        issues.append(f'{path} must be a finite number')
        return False
    if minimum is not None and float(value) < minimum:
        issues.append(f'{path} must be >= {minimum}')
        return False
    return True


def _node_resources(node: Mapping[str, Any]) -> Set[str]:
    node_type = node.get('type')
    if node_type == 'action':
        target = node.get('target')
        return {target} if target in TARGETS else set()
    key = 'children' if node_type == 'sequence' else 'branches'
    resources: Set[str] = set()
    for child in node.get(key, []) if isinstance(node.get(key), list) else []:
        if isinstance(child, Mapping):
            resources.update(_node_resources(child))
    return resources


def show_resources(document: Mapping[str, Any]) -> Set[str]:
    """Return all devices used by a validated show."""
    show = document.get('show', {})
    root = show.get('root', {}) if isinstance(show, Mapping) else {}
    return _node_resources(root) if isinstance(root, Mapping) else set()


def validate_show(document: Any) -> None:
    """Validate schema, arguments, resource conflicts, and rail transitions."""
    issues: list[str] = []
    if not isinstance(document, Mapping):
        raise ShowValidationError(['YAML root must be a mapping'])
    if str(document.get('schema_version', '')) != '1.0':
        issues.append('schema_version must be "1.0"')

    show = document.get('show')
    if not isinstance(show, Mapping):
        raise ShowValidationError(issues + ['show must be a mapping'])
    if not isinstance(show.get('id'), str) or not show.get('id', '').strip():
        issues.append('show.id must be a non-empty string')

    defaults = show.get('defaults', {})
    if defaults is None:
        defaults = {}
    if not isinstance(defaults, Mapping):
        issues.append('show.defaults must be a mapping')
        defaults = {}
    default_timeout = defaults.get('command_timeout_s', 30.0)
    _require_number(default_timeout, 'show.defaults.command_timeout_s', issues, 0.001)
    if defaults.get('on_failure', 'abort_show') != 'abort_show':
        issues.append('first version only supports defaults.on_failure: abort_show')

    requirements = show.get('requirements', {})
    if requirements is None:
        requirements = {}
    if not isinstance(requirements, Mapping):
        issues.append('show.requirements must be a mapping')
        requirements = {}

    rail_states: Dict[str, str | None] = {}
    for rail in sorted(RAILS):
        initial = requirements.get(f'{rail}_initial_state')
        if initial is not None:
            initial = str(initial).upper()
            if initial not in {'TOP', 'BOTTOM'}:
                issues.append(
                    f'show.requirements.{rail}_initial_state must be TOP or BOTTOM'
                )
                initial = None
        rail_states[rail] = initial

    root = show.get('root')
    if not isinstance(root, Mapping):
        raise ShowValidationError(issues + ['show.root must be a workflow node'])

    def validate_node(
        node: Any, path: str, states: MutableMapping[str, str | None]
    ) -> Set[str]:
        if not isinstance(node, Mapping):
            issues.append(f'{path} must be a mapping')
            return set()

        node_path = source_reference(node, path)
        node_id = node.get('id')
        if node_id is not None and (
            not isinstance(node_id, str) or not node_id.strip()
        ):
            issues.append(f'{node_path}.id must be a non-empty string when provided')

        node_type = node.get('type')
        if node_type == 'action':
            return validate_action(node, node_path, states)
        if node_type == 'delay':
            _require_number(
                node.get('duration_s'),
                f'{node_path}.duration_s',
                issues,
                0.0,
            )
            if node.get('duration_s') == 0:
                issues.append(f'{node_path}.duration_s must be greater than zero')
            return set()
        if node_type not in {'sequence', 'parallel'}:
            issues.append(
                f'{node_path}.type must be action, sequence, parallel, or delay'
            )
            return set()

        child_key = 'children' if node_type == 'sequence' else 'branches'
        children = node.get(child_key)
        if not isinstance(children, list) or not children:
            issues.append(f'{node_path}.{child_key} must be a non-empty list')
            return set()
        if node_type == 'parallel' and node.get('join', 'all') != 'all':
            issues.append(f'{node_path}.join must be all')

        resources: Set[str] = set()
        if node_type == 'sequence':
            for index, child in enumerate(children):
                resources.update(
                    validate_node(child, f'{path}.{child_key}[{index}]', states)
                )
            return resources

        branch_resources: list[Set[str]] = []
        branch_states: list[MutableMapping[str, str | None]] = []
        for index, child in enumerate(children):
            child_states = deepcopy(states)
            used = validate_node(
                child, f'{path}.{child_key}[{index}]', child_states
            )
            for previous_index, previous in enumerate(branch_resources):
                overlap = used & previous
                if overlap:
                    issues.append(
                        f'{node_path} has parallel resource conflict between branches '
                        f'{previous_index} and {index}: {", ".join(sorted(overlap))}'
                    )
            branch_resources.append(used)
            branch_states.append(child_states)
            resources.update(used)

        for used, child_states in zip(branch_resources, branch_states):
            for rail in RAILS & used:
                states[rail] = child_states[rail]
        return resources

    def validate_action(
        node: Mapping[str, Any],
        path: str,
        states: MutableMapping[str, str | None],
    ) -> Set[str]:
        target = node.get('target')
        command = node.get('command')
        args = node.get('args', {})
        if target not in TARGETS:
            issues.append(f'{path}.target is unsupported: {target}')
            return set()
        if not isinstance(args, Mapping):
            issues.append(f'{path}.args must be a mapping')
            args = {}
        timeout = node.get('timeout_s', default_timeout)
        _require_number(timeout, f'{path}.timeout_s', issues, 0.001)

        if target == 'arm':
            if command != 'joint_move':
                issues.append(f'{path}: arm only supports command joint_move')
            positions = args.get('positions')
            if not isinstance(positions, list) or len(positions) != 6:
                issues.append(f'{path}.args.positions must contain exactly 6 values')
            elif not all(_is_number(position) for position in positions):
                issues.append(f'{path}.args.positions must contain finite numbers')
            speed = args.get('speed_scaling')
            if _require_number(speed, f'{path}.args.speed_scaling', issues):
                if not 0.2 <= float(speed) <= 1.0:
                    issues.append(
                        f'{path}.args.speed_scaling must be between 0.2 and 1.0'
                    )

        elif target in RAILS:
            if command != 'move':
                issues.append(f'{path}: {target} only supports command move')
            steps = args.get('steps')
            if (
                not isinstance(steps, int)
                or isinstance(steps, bool)
                or steps != 16000
            ):
                issues.append(
                    f'{path}.args.steps must be 16000 for the two-position rail model'
                )
            direction = str(args.get('direction', '')).lower()
            if direction not in {'low', 'high'}:
                issues.append(f'{path}.args.direction must be low or high')
            else:
                expected = 'TOP' if direction == 'low' else 'BOTTOM'
                destination = 'BOTTOM' if direction == 'low' else 'TOP'
                if states[target] is None:
                    issues.append(
                        f'{path}: {target} position is UNKNOWN; declare a trusted '
                        f'{target}_initial_state'
                    )
                elif states[target] != expected:
                    issues.append(
                        f'{path}: illegal {target} {direction} move from {states[target]}'
                    )
                else:
                    states[target] = destination

        elif target in {'servo1', 'servo2'}:
            if command != 'move_to':
                issues.append(f'{path}: {target} only supports command move_to')
            degree = args.get('target_degree')
            if not isinstance(degree, int) or isinstance(degree, bool):
                issues.append(f'{path}.args.target_degree must be an integer')
            elif _require_number(degree, f'{path}.args.target_degree', issues):
                if not 0 <= float(degree) <= 180:
                    issues.append(
                        f'{path}.args.target_degree must be between 0 and 180'
                    )
            delay = args.get('step_delay_ms')
            if not isinstance(delay, int) or isinstance(delay, bool):
                issues.append(f'{path}.args.step_delay_ms must be an integer')
            elif _require_number(delay, f'{path}.args.step_delay_ms', issues):
                if not 1 <= int(delay) <= 10000:
                    issues.append(
                        f'{path}.args.step_delay_ms must be between 1 and 10000'
                    )

        elif target == 'screen':
            if command != 'face':
                issues.append(f'{path}: screen only supports command face')
            face_type = str(args.get('face_type', '')).upper()
            if face_type not in FACE_TYPES:
                issues.append(f'{path}.args.face_type is unsupported: {face_type}')

        return {target}

    validate_node(root, 'show.root', rail_states)
    if issues:
        raise ShowValidationError(issues)
