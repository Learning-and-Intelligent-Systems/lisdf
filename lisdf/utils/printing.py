from typing import Optional

__all__ = ["DEFAULT_TABSIZE", "indent_text"]


DEFAULT_TABSIZE = 2


def indent_text(
    text: str,
    level: int = 1,
    indent_format: Optional[str] = None,
    tabsize: Optional[int] = None,
):
    if indent_format is not None:
        assert tabsize is None, "Cannot provide both indent format and tabsize."
    if tabsize is not None:
        assert indent_format is None, "Cannot provide both indent format and tabsize."
        indent_format = " " * tabsize
    if indent_format is None and tabsize is None:
        indent_format = "  "
    assert isinstance(indent_format, str)
    indent_format = indent_format * level
    return indent_format + text.replace("\n", "\n" + indent_format)
