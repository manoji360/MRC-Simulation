"""Errors meant to be shown to end users.

Anything raised as a PipelineError is printed by the CLI as a plain message (plus an
optional hint on how to fix it), without a Python traceback. Any other exception is a
bug in the tool.
"""

from __future__ import annotations


class PipelineError(Exception):
    """A problem the user can fix (bad input, missing file, bad config)."""

    def __init__(self, message: str, hint: str | None = None) -> None:
        super().__init__(message)
        self.hint = hint


class ValidationError(PipelineError):
    """The URDF is structurally invalid. `problems` lists every problem found."""

    def __init__(self, problems: list[str], source: str) -> None:
        self.problems = list(problems)
        super().__init__(
            f"{source} is not a valid URDF ({len(self.problems)} problem(s) found).",
            hint="Fix the problems listed above in the URDF (or its xacro source) and run again.",
        )
