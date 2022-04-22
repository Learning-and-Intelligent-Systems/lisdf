from typing import Optional

DEFAULT_TABSIZE = 2


def indent_text(
    text: str,
    level: int = 1,
    indent_format: Optional[str] = None,
    tabsize: Optional[int] = None,
    strip: bool = True,
):
    if indent_format is not None:
        assert tabsize is None, "Cannot provide both indent format and tabsize."
    if tabsize is not None:
        assert indent_format is None, "Cannot provide both indent format and tabsize."
        indent_format = " " * tabsize
    if indent_format is None and tabsize is None:
        indent_format = " " * DEFAULT_TABSIZE
    assert isinstance(indent_format, str)
    indent_format = indent_format * level

    rv = indent_format + text.replace("\n", "\n" + indent_format)
    return rv if not strip else rv.strip()
