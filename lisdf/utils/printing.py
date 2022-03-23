__all__ = ['DEFAULT_TABSIZE', 'indent_text']


DEFAULT_TABSIZE = 2


def indent_text(text, level=1, indent_format=None, tabsize=None):
    if indent_format is not None:
        assert tabsize is None, 'Cannot provide both indent format and tabsize.'
    if tabsize is not None:
        assert indent_format is None, 'Cannot provide both indent format and tabsize.'
        indent_format = ' ' * tabsize
    if indent_format is None and tabsize is None:
        indent_format = '  '
    indent_format = indent_format * level
    return indent_format + text.replace('\n', '\n' + indent_format)
